import argparse
import numpy as np
import matplotlib.pyplot as plt


def cumulative_rmse(values: np.ndarray) -> np.ndarray:
    sq = values ** 2
    csum = np.cumsum(sq)
    n = np.arange(1, len(values) + 1)
    return np.sqrt(csum / n)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot EKF trajectory and velocity RMSE")
    parser.add_argument("--csv", default="ekf_log.csv", help="Path to EKF log CSV")
    args = parser.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)

    gt_px = data["gt_px"]
    gt_py = data["gt_py"]
    est_px = data["est_px"]
    est_py = data["est_py"]

    gt_vx = data["gt_vx"]
    gt_vy = data["gt_vy"]
    est_vx = data["est_vx"]
    est_vy = data["est_vy"]

    t = data["t"]

    vel_error_mag = np.sqrt((est_vx - gt_vx) ** 2 + (est_vy - gt_vy) ** 2)
    vel_rmse_over_time = cumulative_rmse(vel_error_mag)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].plot(gt_px, gt_py, label="Ground Truth", linewidth=2)
    axes[0].plot(est_px, est_py, "--", label="EKF Estimate", linewidth=2)
    axes[0].set_title("Trajectory: Ground Truth vs EKF")
    axes[0].set_xlabel("x position [m]")
    axes[0].set_ylabel("y position [m]")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, vel_rmse_over_time, color="tab:red", linewidth=2)
    axes[1].set_title("Velocity RMSE Over Time")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("RMSE of velocity [m/s]")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
