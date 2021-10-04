#include "swing_ekf.h"


/*
 * EKF covariance matrices
 */

#define P_INIT      (10.)
#define Q_INIT      (0.001)
#define R_INIT_ACC  (1.)
#define R_INIT_GYRO (0.1)
// P(k=0) state covariance matrix
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
// Q = process noise covariance matrix
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,      0,         0,
                                                0,      Q_INIT, 0,         0,
                                                0,      0,      10*Q_INIT, 0,
                                                0,      0,      0,         10*Q_INIT};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
// R = measurement noise covariance matrix
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {R_INIT_ACC, 0,           0,           0, 
                                                0,          R_INIT_GYRO, 0,           0,
                                                0,          0,           R_INIT_GYRO, 0,
                                                0,          0,           0,           R_INIT_GYRO};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);


/*
 * Constructor
 */

SwingEKF::SwingEKF()
    : Y(SS_Z_LEN, 1)
    , U(SS_U_LEN, 1)
    , X_est_init(SS_X_LEN, 1)
    , ekf( X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT,
           Main_bUpdateNonlinearX, Main_bUpdateNonlinearY,
           Main_bCalcJacobianF, Main_bCalcJacobianH )
{
}

/*
 * Reset internal state
 */

void SwingEKF::reset() {
    X_est_init.vSetToZero();
    X_est_init[0][0] = -3.14159265359/4.;
    ekf.vReset(X_est_init, EKF_PINIT, EKF_QINIT, EKF_RINIT);
}

/*
 * Run Kalman update step, fed sensor input Y
 */

bool SwingEKF::kalmanUpdateStep( float_prec accel_abs, float_prec gyro_x, float_prec gyro_y, float_prec gyro_z ) {
	Y[0][0] = accel_abs;
	Y[1][0] = gyro_x;
	Y[2][0] = gyro_y;
	Y[3][0] = gyro_z;
	return ekf.bUpdate(Y, U);
}


/*
 * Linearization functions
 */


bool SwingEKF::Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     *  The update function in discrete time:
     *      x1(k+1) = x1(k) + x2(k)*dt
     *      x2(k+1) = x2(k) - g/l*sin(x1(k))*dt 
     */
    float_prec theta        = X[0][0];
    float_prec theta_dot    = X[1][0];
    float_prec rotation     = X[2][0];
    float_prec rotation_dot = X[3][0];
    
    if (theta > 3.14159265359) {
        theta -= 3.14159265359;
    }
    if (theta < -3.14159265359) {
        theta += 3.14159265359;
    }

    while (rotation > 3.14159265359) {
        rotation -= 2*3.14159265359;
    }
    while (rotation < -3.14159265359) {
        rotation += 2*3.14159265359;
    }
    
    X_Next[0][0] = theta + (theta_dot*Step_size_s);   
    X_Next[1][0] = theta_dot - (Pend_g/Pend_l)*sin(theta)*Step_size_s;
    X_Next[2][0] = rotation + (rotation_dot*Step_size_s);
    X_Next[3][0] = rotation_dot;
    
    return true;
}

bool SwingEKF::Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     *  The output (in discrete time):
     *      y1(k) =  acceleration magnitude = g + l * theta_dot**2 // small-angle approximation
     *      y2(k) =  gyro x
     *      y3(k) =  gyro y
     *      y4(k) =  gyro z = swing rotational speed = x_4
     */
    float_prec theta_dot = X[1][0];
    float_prec rotation_dot = X[3][0];
    
    Y[0][0] = Pend_g + Pend_l * theta_dot*theta_dot;
    Y[1][0] = 0; // TODO
    Y[2][0] = 0; // TODO
    Y[3][0] = rotation_dot;
    
    return true;
}

bool SwingEKF::Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    /*  The update function in discrete time:
     *      x1(k+1) = f1(x,u) = x1(k) + x2(k)*dt
     *      x2(k+1) = f2(x,u) = x2(k) - g/l*sin(x1(k))*dt 
     *      x3(k+1) = f3(x,u) = x3(k) + x4(k)*dt
     *      x4(k+1) = f4(x,u) = x4(k)
     * 
     * 
     *  The Jacobian matrix is a 4x4 matrix
     *      F = [ d(f1)/dx1  d(f1)/dx2  d(f1)/dx3  d(f1)/dx4 ]
     *          [ d(f2)/dx1  d(f2)/dx2  d(f2)/dx3  d(f2)/dx4 ]
     *          [ d(f3)/dx1  d(f3)/dx2  d(f3)/dx3  d(f3)/dx4 ]
     *          [ d(f4)/dx1  d(f4)/dx2  d(f4)/dx3  d(f4)/dx4 ]
     * 
     *      F = [1                     dt   0  0 ]
     *          [-g/l*cos(x2(k))*dt    1    0  0 ]
     *          [0                     0    1  dt]
     *          [0                     0    0   1]
     * 
     */
    float_prec theta     = X[0][0];

    F[0][0] =  1.000;
    F[0][1] =  Step_size_s;
    F[0][2] =  0; 
    F[0][3] =  0;

    F[1][0] =  -Pend_g/Pend_l*cos(theta)*Step_size_s;
    F[1][1] =  1.000;
    F[1][2] =  0;
    F[1][3] =  0;

    F[2][0] = 0;
    F[2][1] = 0;
    F[2][2] = 1.000;
    F[2][3] = Step_size_s;

    F[3][0] = 0;
    F[3][1] = 0;
    F[3][2] = 0;
    F[3][3] = 1.000;
    
    return true;
}

bool SwingEKF::Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    /*  The output (in discrete time):
     *      y1(k) = h1(x) = acceleration magnitude = g + l * theta_dot**2 // small-angle approximation
     *      y2(k) = h2(x) = gyro x
     *      y3(k) = h3(x) = gyro y
     *      y4(k) = h4(x) = gyro z
     * 
     * 
     *  The Jacobian matrix is 2x2 matrix (because we have 2 outputs):
     *      H = [d(h1)/dx1    d(h1)/dx2    d(h1)/dx3    d(h1)/dx4]
     *          [d(h2)/dx1    d(h2)/dx2    d(h2)/dx3    d(h2)/dx4]
     *          [d(h3)/dx1    d(h3)/dx2    d(h3)/dx3    d(h3)/dx4]
     *          [d(h4)/dx1    d(h4)/dx2    d(h4)/dx3    d(h4)/dx4]
     * 
     *      H = [0  cos(x1(k)) * l   0    0]
     *          [0  0                0    0]
     *          [0  0                0    0]
     *          [0  0                0    1]
     * 
     */
    float_prec theta_dot = X[1][0];

    H[0][0] = 0;
    H[0][1] = 2 * Pend_l * theta_dot;
    H[0][2] = 0;
    H[0][3] = 0;
    
    H[1][0] = 0;
    H[1][1] = 0;
    H[1][2] = 0;
    H[1][3] = 0;

    H[2][0] = 0;
    H[2][1] = 0;
    H[2][2] = 0;
    H[2][3] = 0;

    H[3][0] = 0;
    H[3][1] = 0;
    H[3][2] = 0;
    H[3][3] = 1;
    
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


