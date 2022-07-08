#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <Eigen/Dense>

template <int N, int M> // state dimension and measurement dimension

class StateEstimator
{
protected:
    StateEstimator(const Eigen::Vector<double, N> &x_init) : x(x_init) {}
    Eigen::Vector<double, N> x;
    
public:
    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::Vector<double, M> &z, const Eigen::Matrix<double, M, M> &R) = 0;
    virtual Eigen::Vector<double, N> getState() = 0;
};

#endif // STATE_ESTIMATOR_H