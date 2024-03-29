#include "kalman_filter.h"
#include <eigen3/Eigen/Dense>

KalmanFilter::KalmanFilter(unsigned int num_states, unsigned int num_obs) :
        num_states_(num_states), num_obs_(num_obs) {
    /*** Predict ***/
    // State vector
    x_ = Eigen::VectorXd::Zero(num_states);
    // Predicted(a prior) state vector
    x_predict_ = Eigen::VectorXd::Zero(num_states);

    // State transition matrix F_
    F_ = Eigen::MatrixXd::Zero(num_states, num_states);

    // Error covariance matrix P
    P_ = Eigen::MatrixXd::Zero(num_states, num_states);
    // Predicted(a prior) error covariance matrix
    P_predict_ = Eigen::MatrixXd::Zero(num_states, num_states);

    // Covariance matrix of process noise
    Q_ = Eigen::MatrixXd::Zero(num_states, num_states);

    /*** Update ***/
    // Observation matrix
    H_ = Eigen::MatrixXd::Zero(num_obs, num_states);

    // Covariance matrix of observation noise
    R_ = Eigen::MatrixXd::Zero(num_obs, num_obs);

    log_likelihood_delta_ = 0.0;
    //noise_ax = noise_ay = noise_aw = noise_ah = 0.0;
    NIS_ = 0.0;
}


void KalmanFilter::Coast() {
    //Eigen::MatrixXd a(10,10);
    /*a<<
              1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 1;*/
    x_predict_ = F_* x_;
    P_predict_ = F_ * P_ * F_.transpose() + Q_;
}


void KalmanFilter::Predict() {
    Coast();
    x_ = x_predict_;
    P_ = P_predict_;

}


Eigen::VectorXd KalmanFilter::PredictionToObservation(const Eigen::VectorXd &state) {
    Eigen::MatrixXd a(10, 1);
    /*a(0, 0) = generateGaussianNoise(0, sqrt(1)),
    a(1, 0) = generateGaussianNoise(0, sqrt(1)),
    a(2, 0) = generateGaussianNoise(0, sqrt(1)),
    a(3, 0) = generateGaussianNoise(0, sqrt(1)),
    a(4, 0) = generateGaussianNoise(0, sqrt(1)),
    a(5, 0) = generateGaussianNoise(0, sqrt(1)),
    a(6, 0) = generateGaussianNoise(0, sqrt(1)),
    a(7, 0) = generateGaussianNoise(0, sqrt(1)),
    a(8, 0) = generateGaussianNoise(0, sqrt(1)),
    a(9, 0) = generateGaussianNoise(0, sqrt(1));*/
    return (H_*state);
}


void KalmanFilter::Update(const Eigen::VectorXd& z) {
    Eigen::VectorXd z_predict = PredictionToObservation(x_predict_);

    // y - innovation, z - real observation, z_predict - predicted observation
    Eigen::VectorXd y = z - z_predict;

    Eigen::MatrixXd Ht = H_.transpose();

    // S - innovation covariance
    Eigen::MatrixXd S = H_ * P_predict_ * Ht + R_;

    NIS_ = y.transpose() * S.inverse() * y;

    // K - Kalman gain
    Eigen::MatrixXd K = P_predict_ * Ht * S.inverse();

    // Updated state estimation
    x_ = x_predict_ + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_states_, num_states_);
    // Joseph form
    //P_ = (I - K * H_) * P_predict_ * (I - K * H_).transpose() + K * R_ * K.transpose();
    // Optimal gain
    P_ = (I - K * H_) * P_predict_;
}


float KalmanFilter::CalculateLogLikelihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& S) {
    float log_likelihood;

    // Note: Computing log(M.determinant()) in Eigen C++ is risky for large matrices since it may overflow or underflow.
    // compute the Cholesky decomposition of the innovation covariance matrix, because it is symmetric
    // S = L * L^T = U^T * U
    // then retrieve factor L in the decomposition
    auto& L = S.llt().matrixL();

    // find log determinant of innovation covariance matrix
    float log_determinant = 0;
    for (unsigned int i = 0; i < S.rows(); i++)
        log_determinant += log(L(i, i));
    log_determinant *= 2;

    // log-likelihood expression for current iteration
    log_likelihood = -0.5 * (y.transpose() * S.inverse() * y + num_obs_ * log(2 * M_PI) + log_determinant);

    if (std::isnan(log_likelihood)) {
        log_likelihood = -1e50;
        }

        return log_likelihood;
}
double KalmanFilter::generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;
    static double z0, z1;
    static bool generate;
    generate = !generate;
    if (!generate)
        return z1 * sigma + mu;
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while (u1 <= epsilon);
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}
