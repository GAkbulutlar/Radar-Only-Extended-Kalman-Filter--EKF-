#include "kalman_filter.h"

#include <cmath>
#include <iostream>

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kPi = 3.14159265358979323846;
}

KalmanFilter::KalmanFilter()
    : is_initialized_(false),
      x_(Eigen::Vector4d::Zero()),
      P_(Eigen::Matrix4d::Identity()),
      F_(Eigen::Matrix4d::Identity()),
      Q_(Eigen::Matrix4d::Zero()),
      R_radar_(Eigen::Matrix3d::Zero()) {
    R_radar_ << 0.09, 0.0, 0.0,
                0.0, 0.0009, 0.0,
                0.0, 0.0, 0.09;
}

void KalmanFilter::Init(const Eigen::Vector4d& x0, const Eigen::Matrix4d& P0) {
    x_ = x0;
    P_ = P0;
    F_ = Eigen::Matrix4d::Identity();
    Q_.setZero();
    is_initialized_ = true;
}

void KalmanFilter::Predict(double dt, double noise_ax, double noise_ay) {
    if (!is_initialized_) {
        return;
    }

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;

    F_ = Eigen::Matrix4d::Identity();
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    Q_.setZero();
    Q_(0, 0) = dt4 / 4.0 * noise_ax;
    Q_(0, 2) = dt3 / 2.0 * noise_ax;
    Q_(1, 1) = dt4 / 4.0 * noise_ay;
    Q_(1, 3) = dt3 / 2.0 * noise_ay;
    Q_(2, 0) = dt3 / 2.0 * noise_ax;
    Q_(2, 2) = dt2 * noise_ax;
    Q_(3, 1) = dt3 / 2.0 * noise_ay;
    Q_(3, 3) = dt2 * noise_ay;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateRadar(const Eigen::Vector3d& z) {
    if (!is_initialized_) {
        return;
    }

    const double px = x_(0);
    const double py = x_(1);
    const double vx = x_(2);
    const double vy = x_(3);

    double rho = std::sqrt(px * px + py * py);
    if (rho < kEpsilon) {
        rho = kEpsilon;
    }

    const double phi = std::atan2(py, px);
    const double rho_dot = (px * vx + py * vy) / rho;

    Eigen::Vector3d z_pred;
    z_pred << rho, phi, rho_dot;

    Eigen::Vector3d y = z - z_pred;
    y(1) = NormalizeAngle(y(1));

    const Eigen::Matrix<double, 3, 4> Hj = CalculateJacobian(x_);
    const Eigen::Matrix3d S = Hj * P_ * Hj.transpose() + R_radar_;
    const Eigen::Matrix<double, 4, 3> K = P_ * Hj.transpose() * S.inverse();

    x_ = x_ + K * y;

    const Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P_ = (I - K * Hj) * P_;
}

Eigen::Matrix<double, 3, 4> KalmanFilter::CalculateJacobian(const Eigen::Vector4d& x_state) const {
    const double px = x_state(0);
    const double py = x_state(1);
    const double vx = x_state(2);
    const double vy = x_state(3);

    const double c1 = px * px + py * py;
    if (c1 < kEpsilon) {
        std::cerr << "Warning: Jacobian division by near-zero avoided. Returning zero Jacobian.\n";
        return Eigen::Matrix<double, 3, 4>::Zero();
    }

    const double c2 = std::sqrt(c1);
    const double c3 = c1 * c2;

    Eigen::Matrix<double, 3, 4> Hj;
    Hj << px / c2, py / c2, 0.0, 0.0,
          -py / c1, px / c1, 0.0, 0.0,
          py * (vx * py - vy * px) / c3,
          px * (vy * px - vx * py) / c3,
          px / c2,
          py / c2;

    return Hj;
}

double KalmanFilter::NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}
