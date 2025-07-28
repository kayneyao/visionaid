#!/usr/bin/env python3
import numpy as np
import allantools
import csv

# csv files
RAW_CSV  = "sensors_raw.csv"
FILT_CSV = "sensors_filtered.csv"

# sampling parameters
TAU0 = 0.01        # sampling interval (s)
RATE = 1.0 / TAU0  # sampling rate (Hz)

# scaling constants
G    = 9.80665     # m/s² per g
FS_A = 2.0         # ±2 g
FS_G = 245.0       # ±245 dps
LSB_A = FS_A / 32768.0 * G       # m/s² per LSB
LSB_G = FS_G / 32768.0 * (np.pi/180.0)  # rad/s per LSB
LSB_M = 1.5e-3     # gauss per LSB

def load_and_scale(path):
    """Load CSV, return dict of arrays in SI units."""
    data = np.loadtxt(path, delimiter=",")
    acc = data[:,0:3] * LSB_A
    gyr = data[:,3:6] * LSB_G
    mag = data[:,6:9] * LSB_M
    return {"acc": acc, "gyr": gyr, "mag": mag}

def compute_arw_bi(name, vec, data_type):
    """
    Given a 1D vector `vec`, compute Allan dev,
    then ARW = adev[0]*sqrt(tau[0]), BI = min(adev).
    """
    taus, adev, _, _ = allantools.oadev(vec, rate=RATE, data_type=data_type)
    arw = adev[0] * np.sqrt(taus[0])
    bi  = np.min(adev)
    return arw, bi

def analyze_set(label, ds):
    print(f"\n--- {label} ---")
    print(f"{'Axis':<5}  {'ARW':>10}  {'BI':>10}")
    for cat, dtype in [("acc","phase"), ("gyr","freq"), ("mag","phase")]:
        for i, axis in enumerate(["X","Y","Z"]):
            vec = ds[cat][:,i]
            arw, bi = compute_arw_bi(f"{cat}{axis}", vec, dtype)
            print(f"{cat[0]}{axis:<3}  {arw:10.3e}  {bi:10.3e}")

def main():
    raw_ds  = load_and_scale(RAW_CSV)
    filt_ds = load_and_scale(FILT_CSV)

    analyze_set("RAW DATA", raw_ds)
    analyze_set("FILTERED DATA", filt_ds)

if __name__ == "__main__":
    main()

# --- RAW DATA ---
# Axis          ARW          BI
# aX     1.080e-03   6.988e-04
# aY     1.058e-01   1.664e-04
# aZ     7.070e-02   1.101e-04
# gX     1.717e-04   7.088e-05
# gY     3.576e-05   2.150e-05
# gZ     3.355e-05   7.862e-06
# mX     6.314e-02   8.951e-05
# mY     7.441e-02   9.372e-05
# mZ     7.339e-02   8.981e-05

# --- FILTERED DATA ---
# Axis          ARW          BI
# aX     9.132e-04   4.510e-04
# aY     1.206e-01   4.341e-04
# aZ     9.854e-02   3.045e-04
# gX     6.764e-04   3.414e-04
# gY     1.017e-04   3.188e-05
# gZ     1.084e-04   8.218e-06
# mX     8.763e-02   2.595e-04
# mY     4.050e-02   6.605e-05
# mZ     4.592e-02   7.646e-05