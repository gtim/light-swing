/*************************************************************************************************************
 * 
 *  Based on pronenewbits EKF library: https://github.com/pronenewbits/Embedded_EKF_Library
 * 
 * 
 * The model then can be described in state space formulation as:
 *  The state variables:
 *      x1 = theta        --> dx1/dt = dtheta/dt
 *      x2 = dtheta/dt    --> dx2/dt = (dtheta)^2)/dt^2
 *  The output variables:
 *      y1 = x
 *      y2 = y
 * 
 *  The update function in continuous time:
 *      dx1/dt = x2
 *      dx2/dt = -g/l * sin(x1)
 *  The update function in discrete time:
 *      x1(k+1) = x1(k) + x2(k)*dt
 *      x2(k+1) = x2(k) - g/l*sin(x1(k))*dt
 * 
 ************************************************************************************************************/
// EKF includes
#include <Wire.h>
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"

// MPU includes
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Bluetooth includes
#include <SoftwareSerial.h>  


// MPU

Adafruit_MPU6050 mpu;


// Bluetooth

int bluetoothTx = 0;
int bluetoothRx = 1;
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);


/* =============================================== The pendulum model constants =============================================== */
#define pend_g      (10.1)             /* acceleration measured at rest (ms^-2) */
#define pend_l      (2.15)             /* length of the pendulum rod, in meters */



/* ============================================ EKF variables/function declaration ============================================ */
/* Just example; in konfig.h: 
 *  SS_X_LEN = 2
 *  SS_Z_LEN = 2
 *  SS_U_LEN = 0 
 */
/* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (100.)
#define Q_INIT      (0.1)
#define R_INIT      (1.)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,
                                                0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,
                                                0,      Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT, 0,
                                                0,      R_INIT};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);
/* EKF variables ---------------------------------------------------------------------------------------------------- */
Matrix X_est_init(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* EKF system declaration ------------------------------------------------------------------------------------------- */
EKF EKF_IMU(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);



/* ========================================= Auxiliary variables/function declaration ========================================= */
elapsedMillis timerLed, timerEKF;
uint64_t u64compuTime;
char bufferTxSer[100];



void setup() {
    /* serial to display data */
    Serial.begin(115200);
    delay(100);

    /* Bluetooth */

    bluetooth.begin(115200);  // BlueSMiRF defaults to 115200bps

    /* MÅU */

    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    
    X_est_init.vSetToZero();
        
    /* Observe that we set the wrong initial x_estimated value!  (X_UKF(k=0) != X_TRUE(k=0)) */
    X_est_init[0][0] = -3.14159265359/4.;
    
    EKF_IMU.vReset(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT);

    delay(100);

}


void loop() {
    if (timerEKF >= SS_DT_MILIS) {
        timerEKF = 0;
        
        /* ================== Read the sensor data / simulate the system here ================== */

        sensors_event_t a, g, temp; // acceleration, gyro, temperature
        mpu.getEvent(&a, &g, &temp);

        // acceleration magnitude squared
        Y[0][0] = sqrt( a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z ); 
        // second measurement placeholder TODO
        Y[1][0] = sqrt( a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z );
        
        /* ------------------ Read the sensor data / simulate the system here ------------------ */
        
        
        /* ============================= Update the Kalman Filter ============================== */
        u64compuTime = micros();
        if (!EKF_IMU.bUpdate(Y, U)) {
            X_est_init.vSetToZero();
            EKF_IMU.vReset(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT);
            Serial.println("Whoop ");
        }
        u64compuTime = (micros() - u64compuTime);
        /* ----------------------------- Update the Kalman Filter ------------------------------ */
        
        
        /* =========================== Print to serial (for plotting) ========================== */
        /* Print: x1 est, x2 est, y1 */
        snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f %.3f %.3f",
                                                     EKF_IMU.GetX()[0][0] *180/3.141592, EKF_IMU.GetX()[1][0], 
                                                     Y[0][0] );
        bluetooth.print(bufferTxSer);
        bluetooth.print('\n');

        /* --------------------------- Print to serial (for plotting) -------------------------- */
    }
}


bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     *  The update function in discrete time:
     *      x1(k+1) = x1(k) + x2(k)*dt
     *      x2(k+1) = x2(k) - g/l*sin(x1(k))*dt 
     */
    float_prec theta     = X[0][0];
    float_prec theta_dot = X[1][0];
    
    if (theta > 3.14159265359) {
        theta = theta - 3.14159265359;
    }
    if (theta < -3.14159265359) {
        theta = theta + 3.14159265359;
    }
    
    X_Next[0][0] = theta + (theta_dot*SS_DT);   
    X_Next[1][0] = theta_dot - (pend_g/pend_l)*sin(theta)*SS_DT;
    
    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     *  The output (in discrete time):
     *      y1(k) =  acceleration magnitude = g + l * theta_dot**2 // small-angle approximation
     *      y2(k) =  TODO
     */
    float_prec theta     = X[0][0];
    float_prec theta_dot = X[1][0];
    
    Y[0][0] = pend_g + pend_l * theta_dot*theta_dot;
    Y[1][0] = 0;
    
    return true;
}

bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    /*  The update function in discrete time:
     *      x1(k+1) = f1(x,u) = x1(k) + x2(k)*dt
     *      x2(k+1) = f2(x,u) = x2(k) - g/l*sin(x1(k))*dt 
     * 
     * 
     *  The Jacobian matrix is 2x2 matrix (because we have 2 state variables):
     *      F = [d(f1)/dx1    d(f1)/dx2]
     *          [d(f2)/dx1    d(f2)/dx2]
     * 
     *      F = [d(x1(k) + x2(k)*dt)/dx1                                        d(x1(k) + x2(k)*dt)/dx2  ]
     *          [d(x2(k) - g/l*sin(x1(k))*dt - alpha*x2*dt)/dx1    d(x2(k) - g/l*sin(x1(k))*dt )/dx2     ]
     * 
     *      F = [1                          dt    ]
     *          [-g/l*cos(x2(k))*dt          1    ]
     * 
     */
    float_prec theta     = X[0][0];

    F[0][0] =  1.000;
    F[0][1] =  SS_DT;

    F[1][0] =  -pend_g/pend_l*cos(theta)*SS_DT;
    F[1][1] =  1.000;
    
    return true;
}

bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    /*  The output (in discrete time):
     *      y1(k) = h1(x) = acceleration magnitude = g + l * theta_dot**2 // small-angle approximation
     *      y2(k) = h2(x) = TODO
     * 
     * 
     *  The Jacobian matrix is 2x2 matrix (because we have 2 outputs):
     *      H = [d(h1)/dx1    d(h1)/dx2]
     *          [d(h2)/dx1    d(h2)/dx2]
     * 
     *      H = [d(sin(x1(k)) * l)/dx1      d(sin(x1(k)) * l)/dx2 ]
     *          [d(-cos(x1(k)) * l)/dx1     d(-cos(x1(k)) * l)/dx2]
     * 
     *      H = [cos(x1(k)) * l      0]
     *          [sin(x1(k)) * l      0]
     * 
     */
    float_prec theta     = X[0][0];
    float_prec theta_dot = X[1][0];

    H[0][0] = 0;
    H[0][1] = 2 * pend_l * theta_dot;
    
    H[1][0] = 0;
    H[1][1] = 0;
    
    return true;
}





void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}
