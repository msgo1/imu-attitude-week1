import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def simulate_imu(T=20.0, fs=200.0):
    """
    Simple synthetic IMU simulation:
    - True roll/pitch follow smooth motions
    - Gyro measures roll_rate/pitch_rate with bias + noise
    - Accel measures gravity direction in body frame with noise
    """
    dt = 1.0 / fs
    t = np.arange(0, T, dt)
    n = len(t)

    # True attitude (rad): smooth motion
    roll_true  = np.deg2rad(10.0) * np.sin(2*np.pi*0.3*t)
    pitch_true = np.deg2rad(6.0)  * np.sin(2*np.pi*0.2*t + 0.5)

    # True angular rates (rad/s)
    roll_rate_true  = np.gradient(roll_true, dt)
    pitch_rate_true = np.gradient(pitch_true, dt)

    # Gyro bias (rad/s) and noise
    gyro_bias_roll  = np.deg2rad(0.3)   # ~0.3 deg/s bias
    gyro_bias_pitch = np.deg2rad(-0.2)
    gyro_noise_std  = np.deg2rad(0.15)  # noise

    gyro_roll  = roll_rate_true  + gyro_bias_roll  + np.random.normal(0, gyro_noise_std, size=n)
    gyro_pitch = pitch_rate_true + gyro_bias_pitch + np.random.normal(0, gyro_noise_std, size=n)

    # Accelerometer measures gravity (no linear acceleration), in body frame
    # For small angles, typical roll/pitch from accel:
    # roll  = atan2(ay, az)
    # pitch = atan2(-ax, sqrt(ay^2 + az^2))
    g = 9.81
    # Gravity components in body frame for roll/pitch (yaw ignored)
    ax = -g * np.sin(pitch_true) #mk. 중력을 +g쪽으로
    ay = g * np.sin(roll_true) * np.cos(pitch_true)
    az = g * np.cos(roll_true) * np.cos(pitch_true)
    print("az mean:", np.mean(az))
    print("roll_acc(deg) range:", np.rad2deg(np.min(np.arctan2(ay, az))), np.rad2deg(np.max(np.arctan2(ay, az))))
    accel_noise_std = 0.15  # m/s^2
    ax_m = ax + np.random.normal(0, accel_noise_std, size=n)
    ay_m = ay + np.random.normal(0, accel_noise_std, size=n)
    az_m = az + np.random.normal(0, accel_noise_std, size=n)

    return t, roll_true, pitch_true, gyro_roll, gyro_pitch, ax_m, ay_m, az_m, dt

def complementary_filter(t, gyro_roll, gyro_pitch, ax, ay, az, dt, alpha=0.98):
    """
    2-axis complementary filter:
    - integrate gyro to predict roll/pitch
    - compute roll/pitch from accel as correction
    - align accel angles to the prediction branch to avoid +/-pi jumps
    """
    n = len(t)
    roll_est = np.zeros(n)
    pitch_est = np.zeros(n)

    # init from accel (i = 0)
    roll_est[0] = np.arctan2(ay[0], az[0])
    pitch_est[0] = np.arctan2(-ax[0], np.sqrt(ay[0]**2 + az[0]**2))

    for i in range(1, n):
        # gyro integration (prediction)
        roll_pred = roll_est[i-1] + gyro_roll[i] * dt
        pitch_pred = pitch_est[i-1] + gyro_pitch[i] * dt

        # accel-based angles (measurement)
        roll_acc = np.arctan2(ay[i], az[i])
        pitch_acc = np.arctan2(-ax[i], np.sqrt(ay[i]**2 + az[i]**2))

        # ★ branch alignment BEFORE blending
        roll_acc  = roll_pred  + wrap_to_pi(roll_acc  - roll_pred)
        pitch_acc = pitch_pred + wrap_to_pi(pitch_acc - pitch_pred)

        # complementary blend
        roll_est[i] = alpha * roll_pred + (1 - alpha) * roll_acc
        pitch_est[i] = alpha * pitch_pred + (1 - alpha) * pitch_acc

    return roll_est, pitch_est

def wrap_to_pi(x):
    return (x + np.pi) % (2*np.pi) - np.pi

def main():
    np.random.seed(0)  # for reproducibility
    Path("results/figures").mkdir(parents=True, exist_ok=True)

    t, roll_true, pitch_true, gyro_roll, gyro_pitch, ax, ay, az, dt = simulate_imu()

    roll_est, pitch_est = complementary_filter(t, gyro_roll, gyro_pitch, ax, ay, az, dt, alpha=0.98)

    roll_est = np.unwrap(roll_est)
    pitch_est = np.unwrap(pitch_est)
    # Plot attitude estimation result
    plt.figure()
    plt.plot(t, np.rad2deg(roll_true), label="roll true")
    plt.plot(t, np.rad2deg(roll_est), label="roll est")
    plt.xlabel("time [s]")
    plt.ylabel("roll [deg]")
    plt.title("Week1 - Roll estimation (complementary filter)")
    plt.legend()
    plt.tight_layout()
    plt.savefig("results/figures/week1_roll.png", dpi=150)
    plt.close()

    plt.figure()
    plt.plot(t, np.rad2deg(pitch_true), label="pitch true")
    plt.plot(t, np.rad2deg(pitch_est), label="pitch est")
    plt.xlabel("time [s]")
    plt.ylabel("pitch [deg]")
    plt.title("Week1 - Pitch estimation (complementary filter)")
    plt.legend()
    plt.tight_layout()
    plt.savefig("results/figures/week1_pitch.png", dpi=150)
    plt.close()

    print("Saved:")
    print("- results/figures/week1_roll.png")
    print("- results/figures/week1_pitch.png")

if __name__ == "__main__":
    main()