#ifndef PENDULUM_FILTER_H
#define PENDULUM_FILTER_H

#include "konfig.h"
#include "matrix.h"
#include "ekf.h"

// Nonlinear & linearization functions

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);


class SwingEKF
{
public:
    SwingEKF();
    void reset();

    // Run Kalman update (once per time step)
    bool kalmanUpdateStep( float_prec measured_angle, float_prec measured_angular_velocity );

    // Getters for estimated state
    float_prec getEstAngle()           { return ekf.GetX()[0][0]; }
    float_prec getEstAngularVelocity() { return ekf.GetX()[1][0]; }

private:
    Matrix Y;
    Matrix U;
    Matrix X_est_init;
public:
    EKF ekf;
};


#endif
