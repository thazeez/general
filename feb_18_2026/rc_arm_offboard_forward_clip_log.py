#!/usr/bin/env python3
"""
RC-ARMED OFFBOARD FORWARD ONLY + HARD CLIP + TELEMETRY LOGGING (Starling 2 / PX4 ROS2)

- You ARM with RC (script never sends ARM)
- Script streams OFFBOARD keepalive + velocity setpoints
- When it detects ARMED + preflight checks pass:
    - requests OFFBOARD
    - commands forward BODY-x velocity for move_time seconds
- Hard clips commanded speed magnitude to vmax_xy (default 0.1 m/s)
- Logs telemetry to CSV for plotting velocity + acceleration

BODY frame (PX4 FRD):
  bx = forward, by = right, bz = down

We command BODY forward only (bx), rotate to NED using VehicleOdometry quaternion,
then clip in NED to vmax_xy (so it cannot exceed 0.1 m/s in horizontal axes).

Run:
  python3 rc_arm_offboard_forward_clip_log.py --bx 0.1 --move-time 6 --vmax-xy 0.1

Logs:
  /home/root/logs/rc_offboard_forward_<timestamp>.csv
"""

import math
import time
import csv
import os
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus


def qos_best_effort(depth=10):
    q = QoSProfile(depth=depth)
    q.history = HistoryPolicy.KEEP_LAST
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    return q


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_R_body_to_ned(w, x, y, z):
    # v_ned = R * v_body   (FRD -> NED)
    R = [[0.0] * 3 for _ in range(3)]

    R[0][0] = 1.0 - 2.0 * (y * y + z * z)
    R[0][1] = 2.0 * (x * y - z * w)
    R[0][2] = 2.0 * (x * z + y * w)

    R[1][0] = 2.0 * (x * y + z * w)
    R[1][1] = 1.0 - 2.0 * (x * x + z * z)
    R[1][2] = 2.0 * (y * z - x * w)

    R[2][0] = 2.0 * (x * z - y * w)
    R[2][1] = 2.0 * (y * z + x * w)
    R[2][2] = 1.0 - 2.0 * (x * x + y * y)

    return R


def R_times_v(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


class RcArmOffboardForwardClipLog(Node):
    def __init__(self, a):
        super().__init__("rc_arm_offboard_forward_clip_log")

        # loop rate
        self.hz = float(max(1.0, a.hz))
        self.dt = 1.0 / self.hz

        # timing
        self.offboard_prestream = float(max(0.1, a.offboard_prestream))
        self.offboard_repeat_s = float(max(0.05, a.offboard_repeat))
        self.offboard_repeat_count = int(max(1, a.offboard_repeat_count))

        # command
        self.bx_cmd = float(a.bx)         # forward
        self.move_time = float(max(0.0, a.move_time))

        # safety clip
        self.vmax_xy = float(abs(a.vmax_xy))
        self.vmax_z = float(abs(a.vmax_z))

        # odom freshness
        self.stale_odom_s = float(max(0.01, a.stale_odom))

        # state
        self.t_start_wall = time.time()
        self.phase = "prestream"

        self.have_odom = False
        self.last_odom_wall = None
        self.qw, self.qx, self.qy, self.qz = 1.0, 0.0, 0.0, 0.0

        self.have_vs = False
        self.armed = False
        self.preflight_ok = False

        self.offboard_sent_n = 0
        self.t_last_offboard_send = None

        self.t_motion_start = None

        # measured telemetry from odom
        self.x = self.y = self.z = float("nan")
        self.vx_m = self.vy_m = self.vz_m = float("nan")
        self.wx_m = self.wy_m = self.wz_m = float("nan")

        # accel (finite difference of measured velocity)
        self.prev_vx_m = None
        self.prev_vy_m = None
        self.prev_vz_m = None
        self.prev_t = None

        self.ax_m = self.ay_m = self.az_m = float("nan")

        # pubs
        self.pub_cm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_best_effort())
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_best_effort())
        self.pub_vc = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_best_effort())

        # subs
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_cb, qos_best_effort())
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vs_cb, qos_best_effort())

        # logging
        os.makedirs("/home/root/logs", exist_ok=True)
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        self.log_path = a.log_path or f"/home/root/logs/rc_offboard_forward_{ts}.csv"
        self.f = open(self.log_path, "w", newline="")
        self.w = csv.writer(self.f)

        self.w.writerow([
            "t_sec", "phase",
            # BODY command
            "bx_cmd",
            # NED command (pre-clip)
            "vN_cmd_raw", "vE_cmd_raw", "vD_cmd_raw",
            # NED command (clipped, actually sent)
            "vN_sp", "vE_sp", "vD_sp",
            # measured pose/vel/ang_vel from VehicleOdometry
            "x", "y", "z",
            "vx_m", "vy_m", "vz_m",
            "wx_m", "wy_m", "wz_m",
            # measured acceleration (finite diff on measured velocity)
            "ax_m", "ay_m", "az_m",
            # flags
            "odom_age_s",
            "armed", "preflight_ok",
            # quaternion
            "qw", "qx", "qy", "qz",
        ])
        self.f.flush()

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("=== RC-ARM OFFBOARD FORWARD + CLIP + LOG ===")
        self.get_logger().info("ARM with RC. Script will switch to OFFBOARD and command forward motion.")
        self.get_logger().info(f"bx_cmd={self.bx_cmd:.3f} m/s | move_time={self.move_time:.2f}s | vmax_xy={self.vmax_xy:.3f} m/s")
        self.get_logger().info(f"Logging to: {self.log_path}")

    def now_s(self):
        return time.time() - self.t_start_wall

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def odom_cb(self, m: VehicleOdometry):
        # quaternion WXYZ
        w, x, y, z = float(m.q[0]), float(m.q[1]), float(m.q[2]), float(m.q[3])
        n = math.sqrt(w*w + x*x + y*y + z*z)
        if n > 1e-9:
            w, x, y, z = w/n, x/n, y/n, z/n
        self.qw, self.qx, self.qy, self.qz = w, x, y, z

        # telemetry
        self.x = float(m.position[0])
        self.y = float(m.position[1])
        self.z = float(m.position[2])

        self.vx_m = float(m.velocity[0])
        self.vy_m = float(m.velocity[1])
        self.vz_m = float(m.velocity[2])

        self.wx_m = float(m.angular_velocity[0])
        self.wy_m = float(m.angular_velocity[1])
        self.wz_m = float(m.angular_velocity[2])

        # acceleration estimate from measured velocity
        t = self.now_s()
        if self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-3 and (self.prev_vx_m is not None):
                self.ax_m = (self.vx_m - self.prev_vx_m) / dt
                self.ay_m = (self.vy_m - self.prev_vy_m) / dt
                self.az_m = (self.vz_m - self.prev_vz_m) / dt

        self.prev_t = t
        self.prev_vx_m = self.vx_m
        self.prev_vy_m = self.vy_m
        self.prev_vz_m = self.vz_m

        self.have_odom = True
        self.last_odom_wall = time.time()

    def vs_cb(self, m: VehicleStatus):
        self.have_vs = True
        self.armed = (int(m.arming_state) == VehicleStatus.ARMING_STATE_ARMED)
        self.preflight_ok = bool(getattr(m, "pre_flight_checks_pass", False))

    def publish_offboard_mode(self):
        cm = OffboardControlMode()
        cm.timestamp = self.now_us()
        cm.position = False
        cm.velocity = True
        cm.acceleration = False
        cm.attitude = False
        cm.body_rate = False
        self.pub_cm.publish(cm)

    def publish_setpoint_vel_ned(self, vN, vE, vD):
        nan = float("nan")
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = nan
        sp.yawspeed = nan
        sp.velocity = [float(vN), float(vE), float(vD)]
        self.pub_sp.publish(sp)

    def send_offboard(self):
        cmd = VehicleCommand()
        cmd.timestamp = self.now_us()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0  # OFFBOARD
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.from_external = True
        self.pub_vc.publish(cmd)

    def loop(self):
        t = self.now_s()

        # always stream keepalive
        self.publish_offboard_mode()

        # odom freshness
        odom_age = (time.time() - self.last_odom_wall) if self.last_odom_wall else 999.0
        stale_odom = (not self.have_odom) or (odom_age > self.stale_odom_s)

        # Default: zero command
        bx = 0.0
        vN_raw = vE_raw = vD_raw = 0.0
        vN_sp = vE_sp = vD_sp = 0.0

        # Phase machine
        if t < self.offboard_prestream:
            self.phase = "prestream"
        else:
            if (not self.have_vs) or stale_odom:
                self.phase = "waiting_topics"
            elif not self.preflight_ok:
                # You won’t be able to arm anyway if this is false.
                self.phase = "preflight_fail"
            elif not self.armed:
                self.phase = "waiting_rc_arm"
            else:
                # armed: request offboard a few times
                if self.offboard_sent_n < self.offboard_repeat_count:
                    if self.t_last_offboard_send is None or (t - self.t_last_offboard_send) >= self.offboard_repeat_s:
                        self.send_offboard()
                        self.t_last_offboard_send = t
                        self.offboard_sent_n += 1

                if self.t_motion_start is None:
                    self.t_motion_start = t
                    self.get_logger().info("ARMED detected. Starting forward motion (clipped).")

                t_rel = t - self.t_motion_start
                if t_rel <= self.move_time:
                    self.phase = "move_forward"
                    bx = float(self.bx_cmd)  # BODY forward
                else:
                    self.phase = "hold"
                    bx = 0.0

                # BODY -> NED rotation
                R_nb = quat_to_R_body_to_ned(self.qw, self.qx, self.qy, self.qz)
                vN_raw, vE_raw, vD_raw = R_times_v(R_nb, [bx, 0.0, 0.0])

                # HARD CLIP in NED
                vN_sp = clamp(vN_raw, -self.vmax_xy, self.vmax_xy)
                vE_sp = clamp(vE_raw, -self.vmax_xy, self.vmax_xy)
                vD_sp = clamp(vD_raw, -self.vmax_z,  self.vmax_z)

        # publish setpoint (always)
        self.publish_setpoint_vel_ned(vN_sp, vE_sp, vD_sp)

        # log one row
        self.w.writerow([
            f"{t:.6f}", self.phase,
            f"{bx:.6f}",
            f"{vN_raw:.6f}", f"{vE_raw:.6f}", f"{vD_raw:.6f}",
            f"{vN_sp:.6f}",  f"{vE_sp:.6f}",  f"{vD_sp:.6f}",
            f"{self.x:.6f}" if not math.isnan(self.x) else "",
            f"{self.y:.6f}" if not math.isnan(self.y) else "",
            f"{self.z:.6f}" if not math.isnan(self.z) else "",
            f"{self.vx_m:.6f}" if not math.isnan(self.vx_m) else "",
            f"{self.vy_m:.6f}" if not math.isnan(self.vy_m) else "",
            f"{self.vz_m:.6f}" if not math.isnan(self.vz_m) else "",
            f"{self.wx_m:.6f}" if not math.isnan(self.wx_m) else "",
            f"{self.wy_m:.6f}" if not math.isnan(self.wy_m) else "",
            f"{self.wz_m:.6f}" if not math.isnan(self.wz_m) else "",
            f"{self.ax_m:.6f}" if not math.isnan(self.ax_m) else "",
            f"{self.ay_m:.6f}" if not math.isnan(self.ay_m) else "",
            f"{self.az_m:.6f}" if not math.isnan(self.az_m) else "",
            f"{odom_age:.3f}",
            int(self.armed), int(self.preflight_ok),
            f"{self.qw:.8f}", f"{self.qx:.8f}", f"{self.qy:.8f}", f"{self.qz:.8f}",
        ])
        self.f.flush()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--offboard-prestream", type=float, default=1.0)
    p.add_argument("--stale-odom", type=float, default=0.25)

    p.add_argument("--offboard-repeat", type=float, default=0.2)
    p.add_argument("--offboard-repeat-count", type=int, default=5)

    # forward command (BODY-x)
    p.add_argument("--bx", type=float, default=0.1, help="BODY forward m/s (will be clipped by vmax-xy in NED)")
    p.add_argument("--move-time", type=float, default=6.0)

    # hard clip
    p.add_argument("--vmax-xy", type=float, default=0.1, help="max horizontal speed magnitude per-axis in NED")
    p.add_argument("--vmax-z", type=float, default=0.2, help="max vertical speed per-axis in NED")

    # log path
    p.add_argument("--log-path", type=str, default="", help="default writes to /home/root/logs/...csv")

    args = p.parse_args()

    rclpy.init()
    node = RcArmOffboardForwardClipLog(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_setpoint_vel_ned(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            node.f.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
