#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <functional>
#include "StateEstimator.h"

template <int N, int M>
class KalmanFilter: public StateEstimator<N, M>
{
private:
    std::function<Eigen::Matrix<double, N, N>(double)> A_ptr;
    Eigen::Matrix<double, N, N> P;
    Eigen::Matrix<double, N, N> Q;
    Eigen::Matrix<double, M, M> R;
    Eigen::Matrix<double, M, N> H;
    Eigen::Matrix<double, N, M> K;

public:
    KalmanFilter(
        const Eigen::Vector<double, N> &x_init, 
        const Eigen::Matrix<double, N, N> &P_init,
        const Eigen::Matrix<double, N, N> &Q_init,
        const Eigen::Matrix<double, M, M> &R_init,
        std::function<Eigen::Matrix<double, N, N>(double)> A_ptr,
        const Eigen::Matrix<double, M,N> &H
    );

    void Predict(double dt);
    void Update(const Eigen::Vector<double, M> &z);
    void Update(const Eigen::Vector<double, M> &z, 
                const Eigen::Matrix<double, M, M> &R);
    Eigen::Vector<double, N> GetState();
};

template <int N, int M>
KalmanFilter<N, M>::KalmanFilter(
    const Eigen::Vector<double, N> &x_init, 
    const Eigen::Matrix<double, N, N> &P_init,
    const Eigen::Matrix<double, N, N> &Q_init,
    const Eigen::Matrix<double, M, M> &R_init,
    std::function<Eigen::Matrix<double, N, N>(double)> A_ptr,
    const Eigen::Matrix<double, M,N> &H
)
: StateEstimator<N, M>(x_init),
    A_ptr(A_ptr),
    P(P_init),
    Q(Q_init),
    R(R_init),
    H(H),
    K(Eigen::Matrix<double, N, M>::Zero())
{
}

template <int N, int M>
void KalmanFilter<N, M>::Predict(double dt)
{
    Eigen::Matrix<double, N, N>A = A_ptr(dt);
    this->x = A * this->x;
    P = A * P * A.transpose() + Q;
}

template <int N, int M>
void KalmanFilter<N, M>::Update(const Eigen::Vector<double, M> &z)
{
    Update(z, R);
}

template <int N, int M>
void KalmanFilter<N, M>::Update(const Eigen::Vector<double, M> &z, const Eigen::Matrix<double, M, M> &R)
{
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    this->x = this->x + K * (z - H * this->x);
    P = P - K * H * P;
}

template <int N, int M>
Eigen::Vector<double, N> KalmanFilter<N, M>::GetState()
{
    return this->x;
}

#endif // KALMAN_FILTER_H