#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();

    void Init(const Eigen::Vector4d& x0, const Eigen::Matrix4d& P0);
    void Predict(double dt, double noise_ax, double noise_ay);
    void UpdateRadar(const Eigen::Vector3d& z);

    const Eigen::Vector4d& state() const { return x_; }
    const Eigen::Matrix4d& covariance() const { return P_; }

private:
    Eigen::Matrix<double, 3, 4> CalculateJacobian(const Eigen::Vector4d& x_state) const;
    static double NormalizeAngle(double angle);

    bool is_initialized_;
    Eigen::Vector4d x_;
    Eigen::Matrix4d P_;
    Eigen::Matrix4d F_;
    Eigen::Matrix4d Q_;
    Eigen::Matrix3d R_radar_;
};
