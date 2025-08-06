#!/usr/bin/env python3
import csv
import numpy as np
import allantools
import yaml

# ——— CONFIGURATION ———
CSV_FILE     = './calibration/imu/imu_log.csv'   # your logged data
FS           = 100.0           # sampling rate [Hz]
OUTPUT_YAML  = 'imu.yaml'      # Kalibr IMU config output
# ————————————————————

# Sensor conversion constants
ACC_FS    = 2.0         # ±2 g
ACC_SENS  = ACC_FS / 32768.0   # g/count
GYRO_FS   = 245.0       # ±245 dps
GYRO_SENS = GYRO_FS / 32768.0  # dps/count
G         = 9.80665     # m/s² per g
D2R       = np.pi / 180.0

# 1) Read CSV (skip header) and collect raw counts
ax_counts = []
ay_counts = []
az_counts = []
gx_counts = []
gy_counts = []
gz_counts = []

with open(CSV_FILE, newline='') as f:
    reader = csv.reader(f)
    next(reader, None)      # skip header
    for row in reader:
        if len(row) < 7:
            continue
        ax_counts.append(int(row[1]))
        ay_counts.append(int(row[2]))
        az_counts.append(int(row[3]))
        gx_counts.append(int(row[4]))
        gy_counts.append(int(row[5]))
        gz_counts.append(int(row[6]))

# 2) Convert to SI
ax = np.array(ax_counts) * ACC_SENS * G
ay = np.array(ay_counts) * ACC_SENS * G
az = np.array(az_counts) * ACC_SENS * G

gx = np.array(gx_counts) * GYRO_SENS * D2R
gy = np.array(gy_counts) * GYRO_SENS * D2R
gz = np.array(gz_counts) * GYRO_SENS * D2R

# 3) Helper to get taus & adev
def allan_params(data, fs):
    taus, adev, _, _ = allantools.oadev(data, rate=fs, data_type='freq')
    return taus, adev

# 4) Gyro Allan → noise & bias
taus_g, adev_g = allan_params(gx, FS)
# noise density @ 1s
idx1 = np.argmin(np.abs(taus_g - 1.0))
n_rad_g = adev_g[idx1]
n_deg_g = n_rad_g * (180.0/np.pi)
# bias instability
idx_min_g = np.nanargmin(adev_g)
sigma_bias_g = adev_g[idx_min_g]
b_rad_g = sigma_bias_g * np.sqrt(2*np.log(2)/np.pi)
b_deg_g = b_rad_g * (180.0/np.pi)

# 5) Accel Allan → noise & bias (compute per-axis, then average)
def accel_stats(data, name):
    taus, adev = allan_params(data, FS)
    idx1 = np.argmin(np.abs(taus - 1.0))
    n = adev[idx1]                     # [m/s²/√Hz]
    idxm = np.nanargmin(adev)
    sigma_bias = adev[idxm]
    b = sigma_bias * np.sqrt(2*np.log(2)/np.pi)  # [m/s²]
    print(f"{name}:")
    print(f"  Noise density:    {n:.4f} m/s²/√Hz   ({n/G*1e3:.2f} mg/√Hz)")
    print(f"  Bias instability: {b:.4f} m/s²      ({b/G*1e3:.2f} mg) at τ = {taus[idxm]:.3f}s")
    return n, b

print("Mahony filter parameters (gyro):")
print(f"  Noise density nₒ:  {n_deg_g:.4f} °/s/√Hz   ({n_rad_g:.4f} rad/s/√Hz)")
print(f"  Bias instability bₒ: {b_deg_g:.4f} °/s   ({b_rad_g:.4f} rad/s) at τ = {taus_g[idx_min_g]:.3f}s\n")

print("Mahony filter parameters (accel):")
n_ax, b_ax = accel_stats(ax, "  X-axis")
n_ay, b_ay = accel_stats(ay, "  Y-axis")
n_az, b_az = accel_stats(az, "  Z-axis")
n_accel = np.mean([n_ax, n_ay, n_az])
b_accel = np.mean([b_ax, b_ay, b_az])
print(f"\n  → Overall accel noise density:    {n_accel:.4f} m/s²/√Hz   ({n_accel/G*1e3:.2f} mg/√Hz)")
print(f"  → Overall accel bias instability: {b_accel:.4f} m/s²      ({b_accel/G*1e3:.2f} mg)")

# 6) (unchanged) Kalibr YAML dump…
# ... rest of your script ...
