/*
 *  SwingEKF
 *
 *  Applies the Extended Kalman Filter implementation to the system of a swing
 *  with mounted accelerometer and gyroscope.
 *
 *  Model state X:
 *      x1 = angle of the rope (rad)
 *      x2 = angular velocity of the rope (rad/s)
 *      x3 = rotation of the swing (rad)
 *      x4 = rotational speed of the swing (rad/s)
 *  Observables Y:
 *      y1 = acceleration magnitude
 *      y2 = gyroscope x
 *      y3 = gyroscope y
 *      y4 = gyroscope z
 * 
 *  State update function:
 *      x1(k+1) = x1(k) + x2(k)*dt
 *      x2(k+1) = x2(k) - g/l*sin(x1(k))*dt
 *      
 *  Using pronenewbits' EKF library: https://github.com/pronenewbits/Embedded_EKF_Library
 * 
 * 
 */


#ifndef SWING_EKF_H
#define SWING_EKF_H

#include "konfig.h"
#include "matrix.h"
#include "ekf.h"




// SwingEKF Class

class SwingEKF
{
public:
    SwingEKF();
    void reset();

    // Run Kalman update (once per time step)
    bool kalmanUpdateStep( float_prec measured_accel_abs,
                           float_prec measured_gyro_x, 
                           float_prec measured_gyro_y,
                           float_prec measured_gyro_z );

    // Getters for estimated state
    float_prec getEstRopeAngle()            { return ekf.GetX()[0][0]; }
    float_prec getEstRopeAngularVelocity()  { return ekf.GetX()[1][0]; }
    float_prec getEstSwingRotation()        { return ekf.GetX()[2][0]; }
    float_prec getEstSwingRotationalSpeed() { return ekf.GetX()[3][0]; }

    // Sample time / time step (s)
    static constexpr float_prec Step_size_s  = 30e-3; // 30ms

private:
    // Linearization functions
    static bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
    static bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
    static bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
    static bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);

    Matrix Y;
    Matrix U;
    Matrix X_est_init;
    EKF ekf;
    
    // Acceleration at rest (ms^-2)
    static constexpr float_prec Pend_g = 10.05; 
    // Length of swing (m)
    static constexpr float_prec Pend_l = 2.0; 
};


#endif
