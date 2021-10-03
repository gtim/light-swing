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

    // Getters for estimated state
    float_prec getEstAngle()           { return ekf.GetX()[0][0]; }
    float_prec getEstAngularVelocity() { return ekf.GetX()[1][0]; }

    Matrix Y;
    Matrix U;
private:
    Matrix X_est_init;
public:
    EKF ekf;
};


#endif
