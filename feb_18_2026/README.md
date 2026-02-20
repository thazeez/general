# general – Starling 2 Experiment Logs (Feb 2026)

This repository stores experiment folders and logs collected during Starling 2 (VOXL2 / PX4) testing.
Experiments are organized by date.

## Feb 18, 2026 (`feb_18_2026/`)

Contains scripts and logs for **RC-armed OFFBOARD forward motion** with **hard velocity clipping** and **telemetry logging**.

### Main script
`rc_arm_offboard_forward_clip_log.py`

**What it does**
- You arm the drone using the RC transmitter (the script does NOT arm).
- The script continuously publishes OFFBOARD keepalive messages.
- Once it detects:
  - ARMED state AND
  - preflight checks pass AND
  - fresh odometry,
  it requests OFFBOARD mode.
- Commands **forward BODY-x velocity** (`--bx`) for `--move-time` seconds.
- Rotates BODY (FRD) velocity into **NED** using VehicleOdometry quaternion.
- Applies a **hard clip** to horizontal NED components using `--vmax-xy` (and vertical using `--vmax-z`).
- Logs telemetry (position, velocity, angular velocity, estimated acceleration) to CSV.

### Run
Example:
```bash
python3 rc_arm_offboard_forward_clip_log.py --bx 0.1 --move-time 6 --vmax-xy 0.1

**Important Note:** THis script **does not **command any upward motion at all (i.e there is no place in the script where the drone is told to climb at 0.1m/s for 5s before forward motion)
