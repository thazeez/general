#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt


def load_csv(fname):
    return np.genfromtxt(fname, delimiter=",", names=True, dtype=None, encoding=None)


def accel_from_vel(t, v):
    a = np.full_like(v, np.nan, dtype=float)
    dt = np.diff(t)
    dv = np.diff(v)
    good = dt > 1e-6
    a[1:][good] = dv[good] / dt[good]
    return a


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_rc_log.py logfile.csv")
        sys.exit(1)

    fname = sys.argv[1]
    d = load_csv(fname)

    # time
    t = d["t_sec"].astype(float)

    # measured position
    x = d["x"].astype(float)
    y = d["y"].astype(float)
    z = d["z"].astype(float)

    # measured velocity (from VehicleOdometry)
    vx = d["vx_m"].astype(float)
    vy = d["vy_m"].astype(float)
    vz = d["vz_m"].astype(float)

    # commanded velocity setpoint actually sent (NED)
    vN_sp = d["vN_sp"].astype(float)
    vE_sp = d["vE_sp"].astype(float)
    vD_sp = d["vD_sp"].astype(float)

    # accel (use logged accel if present; otherwise compute)
    if "ax_m" in d.dtype.names and "ay_m" in d.dtype.names and "az_m" in d.dtype.names:
        ax = d["ax_m"].astype(float)
        ay = d["ay_m"].astype(float)
        az = d["az_m"].astype(float)
    else:
        ax = accel_from_vel(t, vx)
        ay = accel_from_vel(t, vy)
        az = accel_from_vel(t, vz)

    # ------- XY -------
    plt.figure()
    plt.plot(x, y)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("XY Trajectory (measured)")
    plt.grid(True)
    plt.axis("equal")

    # ------- XZ -------
    plt.figure()
    plt.plot(x, z)
    plt.xlabel("X [m]")
    plt.ylabel("Z [m]")
    plt.title("XZ Trajectory (measured)")
    plt.grid(True)
    plt.axis("equal")

    # ------- YZ -------
    plt.figure()
    plt.plot(y, z)
    plt.xlabel("Y [m]")
    plt.ylabel("Z [m]")
    plt.title("YZ Trajectory (measured)")
    plt.grid(True)
    plt.axis("equal")

    # ------- 3D -------
    fig = plt.figure()
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(x, y, z)
    ax3.set_xlabel("X [m]")
    ax3.set_ylabel("Y [m]")
    ax3.set_zlabel("Z [m]")
    ax3.set_title("3D Trajectory (measured)")

    # ------- velocity vs time -------
    plt.figure()
    plt.plot(t, vx, label="vx_m (meas)")
    plt.plot(t, vy, label="vy_m (meas)")
    plt.plot(t, vz, label="vz_m (meas)")
    plt.plot(t, vN_sp, "--", label="vN_sp (cmd)")
    plt.plot(t, vE_sp, "--", label="vE_sp (cmd)")
    plt.plot(t, vD_sp, "--", label="vD_sp (cmd)")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Velocity: commanded (NED sp) vs measured (odom)")
    plt.legend()
    plt.grid(True)

    # ------- acceleration vs time -------
    plt.figure()
    plt.plot(t, ax, label="ax_m")
    plt.plot(t, ay, label="ay_m")
    plt.plot(t, az, label="az_m")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s^2]")
    plt.title("Measured Acceleration (finite-difference on odom velocity)")
    plt.legend()
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
