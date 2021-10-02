/*
 * 
 *  Model state X:
 *      x1 = theta = angle of the swing (rad)
 *      x2 = thetadot = angular velocity of the swing (rad/s)
 *  Observables Y:
 *      y1 = acceleration magnitude
 *      y2 = gyroscope x-y magnitude (not implemented)
 * 
 *  State update function:
 *      x1(k+1) = x1(k) + x2(k)*dt
 *      x2(k+1) = x2(k) - g/l*sin(x1(k))*dt
 *      
 *  Using pronenewbits EKF implementation: https://github.com/pronenewbits/Embedded_EKF_Library
 * 
 */
 
// Embedded_EKF_Library
#include <Wire.h> // also for MPU
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"

// MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Bluetooth 
#include <SoftwareSerial.h>

// FastLED
#include "SPI.h"
#include <FastLED.h>

// Pre-recorded sensor data for testing
#include "mock_sensor_data.h"
//#define USE_MOCK_SENSOR_DATA



// MPU
Adafruit_MPU6050 mpu;

// Bluetooth
int bluetoothTx = 0;
int bluetoothRx = 1;
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// FastLED
const uint8_t Num_LEDs = 5;
const uint8_t LED_Data_Pin = 7;
CRGB leds[Num_LEDs];

// Lightshow state variables
bool angvel_positive = true; // track to find for zero-crossing = swing turns
uint32_t last_swingturn_ms = 0; // millis() of last swing turn = angvel zero-crossing
uint32_t last_swingturn_posneg_ms = 0; // angvel pos->neg (arbitrarily left/right)
uint32_t last_swingturn_negpos_ms = 0;

/* 
 *  EKF variables/declarations
 */

// Physical constants
#define pend_g      (10.05)  // acceleration measured at rest (ms^-2)
#define pend_l      (2.0)    // length of swing (m)

// EKF covariance matrices

#define P_INIT      (10.)
#define Q_INIT      (0.001)
#define R_INIT      (1)
// P(k=0) state covariance matrix
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,
                                                0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
// Q = process noise covariance matrix
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,
                                                0,      Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
// R = measurement noise covariance matrix
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT, 0,
                                                0,      R_INIT};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);

// Nonlinear & linearization functions
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);
// EKF variables
Matrix X_est_init(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
// EKF system declaration
EKF EKF_IMU(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);
// EKF auxiliary variables
elapsedMillis timerLed, timerEKF;
char bufferTxSer[100];


/*
 * Setup
 */

void setup() {
  
    // Init serial and Bluetooth
    
    Serial.begin(115200);
    delay(100);
    bluetooth.begin(115200);  // BlueSMiRF defaults to 115200bps

    // MPU/sensor

    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // FastLED init
    
    FastLED.setMaxPowerInVoltsAndMilliamps(5,500);
    FastLED.addLeds<WS2812, LED_Data_Pin>(leds, Num_LEDs);  
    for ( uint8_t i = 0; i < Num_LEDs; i++ )
      leds[i] = CRGB::Black;
    FastLED.show();

    // EKF init
    
    X_est_init.vSetToZero();
    X_est_init[0][0] = -3.14159265359/4.;
    EKF_IMU.vReset(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT);

    delay(100);

}

/*
 * Main loop
 */

void loop() {
    if (timerEKF >= SS_DT_MILIS) {
        timerEKF = 0;
        
        // Read sensor data

        #ifndef USE_MOCK_SENSOR_DATA

          // Use live sensor data
          
          sensors_event_t a, g, temp; // acceleration, gyro, temperature
          mpu.getEvent(&a, &g, &temp);
  
          // acceleration magnitude 
          Y[0][0] = sqrt( a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z ); 
          // gyro x/y magnitude (not including z = twist)
          Y[1][0] = sqrt( g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y );

        #else

          // Use pre-recorded mock data

          Y[0][0] = mock_sensor_data[mock_sensor_data_row_i][0];
          Y[1][0] = mock_sensor_data[mock_sensor_data_row_i][1];
          mock_sensor_data_row_i = ( mock_sensor_data_row_i + 1 ) % MOCK_SENSOR_DATA_NUM_ROWS;
          
        #endif

        
        // Log measurements to bluetooth to use as pre-recorded test case data

        //snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.5f %.5f", Y[0][0], Y[1][0] );
        //bluetooth.print(bufferTxSer);
        //bluetooth.print('\n');
        
        
        // Update Kalman filter
        
        if (!EKF_IMU.bUpdate(Y, U)) {
            X_est_init.vSetToZero();
            EKF_IMU.vReset(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT);
            Serial.println("Whoop ");
        }

        // Update lightshow state

        // Angular velocity crosses zero = swing turns at end of arc
        float_prec angvel_est = EKF_IMU.GetX()[1][0];
        if (          angvel_positive && angvel_est < 0 ) {
          last_swingturn_ms = millis();
          last_swingturn_posneg_ms = last_swingturn_ms;
          angvel_positive = false;
        } else if ( ! angvel_positive && angvel_est > 0 ) {
          last_swingturn_ms = millis();
          last_swingturn_negpos_ms = last_swingturn_ms;
          angvel_positive = true;
        }

        // Update LEDs

        uint32_t cur_millis = millis();
        if ( cur_millis - last_swingturn_ms < 1000 ) {
          //uint8_t hsv_hue   = map( cur_millis - last_swingturn_ms, 0, 1000, 224, 128 );
          //hsv_hue   = constrain( hsv_hue, 128, 224 );
          uint8_t hsv_hue = ( last_swingturn_negpos_ms > last_swingturn_posneg_ms ? 224 : 128 );
          uint8_t hsv_value = map( cur_millis - last_swingturn_ms, 0, 1000, 255, 0 );
          for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
            leds[i] = CHSV(hsv_hue,255,hsv_value);
          }
        } else {
          for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
            leds[i] = CRGB::Black;
          }
        }
        FastLED.show();
        
        // Print state to bluetooth/serial for live plotting
        
        snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f %.3f %.3f %.3f %lu",
                                                     EKF_IMU.GetX()[0][0] *180/3.1415, // x1 = estimated angle (deg)
                                                     EKF_IMU.GetX()[1][0] *180/3.1415, // x2 = estimated angular velocity (deg/s)
                                                     Y[0][0],   // y1 = measured acceleration magnitude
                                                     Y[1][0]*50, // y2 = measured gyro x/y magnitude (times 50)
                                                     (millis()-last_swingturn_ms)/10 // time since last swing turn (ms/10)
        );
        Serial.print(bufferTxSer);
        Serial.print('\n');
        
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
