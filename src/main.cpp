#include "kalman_filter.h"

#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

namespace {
constexpr double kEpsilon = 1e-6;
}

int main() {
    const double dt = 0.1;
    const int steps = 300;

    const double noise_ax = 5.0;
    const double noise_ay = 5.0;

    const double radar_std_rho = 0.3;
    const double radar_std_phi = 0.03;
    const double radar_std_rhodot = 0.3;

    std::mt19937 rng(42);
    std::normal_distribution<double> noise_rho(0.0, radar_std_rho);
    std::normal_distribution<double> noise_phi(0.0, radar_std_phi);
    std::normal_distribution<double> noise_rhodot(0.0, radar_std_rhodot);

    Eigen::Vector4d gt_state;
    gt_state << 0.0, 0.0, 8.0, 3.0;

    KalmanFilter ekf;

    std::ofstream csv("ekf_log.csv");
    if (!csv.is_open()) {
        std::cerr << "Failed to open output CSV file.\n";
        return 1;
    }

    csv << "t,gt_px,gt_py,gt_vx,gt_vy,est_px,est_py,est_vx,est_vy\n";

    bool initialized = false;
    double vel_sq_error_sum = 0.0;

    for (int k = 0; k < steps; ++k) {
        const double t = k * dt;

        gt_state(0) += gt_state(2) * dt;
        gt_state(1) += gt_state(3) * dt;

        const double gt_px = gt_state(0);
        const double gt_py = gt_state(1);
        const double gt_vx = gt_state(2);
        const double gt_vy = gt_state(3);

        const double true_rho = std::sqrt(gt_px * gt_px + gt_py * gt_py);
        const double true_phi = std::atan2(gt_py, gt_px);
        const double true_rhodot = (gt_px * gt_vx + gt_py * gt_vy) / std::max(true_rho, kEpsilon);

        Eigen::Vector3d z;
        z << true_rho + noise_rho(rng),
             true_phi + noise_phi(rng),
             true_rhodot + noise_rhodot(rng);

        if (!initialized) {
            Eigen::Vector4d x0;
            x0 << z(0) * std::cos(z(1)),
                  z(0) * std::sin(z(1)),
                  z(2) * std::cos(z(1)),
                  z(2) * std::sin(z(1));

            Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity();
            P0(0, 0) = 1.0;
            P0(1, 1) = 1.0;
            P0(2, 2) = 500.0;
            P0(3, 3) = 500.0;

            ekf.Init(x0, P0);
            initialized = true;
        } else {
            ekf.Predict(dt, noise_ax, noise_ay);
            ekf.UpdateRadar(z);
        }

        const Eigen::Vector4d est = ekf.state();

        const double dvx = est(2) - gt_vx;
        const double dvy = est(3) - gt_vy;
        vel_sq_error_sum += dvx * dvx + dvy * dvy;

        csv << t << ','
            << gt_px << ',' << gt_py << ',' << gt_vx << ',' << gt_vy << ','
            << est(0) << ',' << est(1) << ',' << est(2) << ',' << est(3) << '\n';
    }

    csv.close();

    const double vel_rmse = std::sqrt(vel_sq_error_sum / static_cast<double>(steps));
    std::cout << "Simulation complete. Results saved to ekf_log.csv\n";
    std::cout << "Velocity RMSE (combined vx, vy): " << vel_rmse << '\n';

    return 0;
}
