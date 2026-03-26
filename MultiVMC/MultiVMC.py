"""
Task-space VMC example (MULTI segment).

Virtual springs and dampers are placed at the end effector of the multi-segment soft robot in Cartesian space.
The resulting forces are mapped to motor torques via J^T.

Usage (from SoftVMC root):
    # from terminal 1 in MultiVMC folder
    cd ros2_dynamixel && colcon build && source install/setup.bash
    sudo echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    ros2 run dynamixel_interface dynamixel_node --ros-args -p baudrate:=1000000 -p motor_ids:=[1,2,3,4,5,6,7,8,9] -p control_mode:=torque

    # terminal 2 in MultiVMC folder
    export PYTHONPATH="${PYTHONPATH}:$(pwd)/MultiVMC"
    python3 MultiVMC.py

    # Optional: go-home script 
"""
import numpy as np
import rclpy
import time
from datetime import datetime
import matplotlib.pyplot as plt

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearDamper, LinearSpring, TanhSpring, TanhDamper, GaussianSpring

from MultiController import MultiController, CONTROL_FREQUENCY
from MultiVMCTaskSpace import VMC_Multi
from FK_Multi import FK_Multi

rclpy.init()
controller = MultiController()

# ------------------ Define waypoints in Cartesian space ------------------
# Positions in Cartesian space [x, y, z] (meters)
POS_SCALE = 3

q1_short = np.array([0.01 * POS_SCALE,                   0.0,                              3 * 0.145 - 0.001])
q2_short = np.array([-0.01 * POS_SCALE * np.sin(np.pi/6), 0.01 * POS_SCALE * np.cos(np.pi/6), 3 * 0.145 - 0.001])
q3_short = np.array([-0.01 * POS_SCALE * np.sin(np.pi/6), -0.01 * POS_SCALE * np.cos(np.pi/6), 3 * 0.145 - 0.001])
rest_pos = np.array([0.0, 0.0, 3 * 0.145])

waypoints      = [q1_short, q2_short, q3_short, rest_pos]
waypoint_index = 0

# ------------------ Control loop constants ------------------
stable_count = -50
STABLE_STEPS = 50
MOTOR_STOPPED_THRESHOLD = 0.1   # [rad/s]
TAU_LIMIT = 0.1                 # [N·m] per motor safety limit

# ------------------- VMC Springs and Dampers ------------------
lin_spring   = LinearSpring(400.0)
tanh_spring  = TanhSpring(200.0, 1.5)
gauss_spring = GaussianSpring(800, 0.03)

lin_damper   = LinearDamper(1.0)
tanh_damper  = TanhDamper(1.0, 1.5)

# Use s_param=1.0 (end of full backbone in your current FK/Jacobian setup)
vmc = VMC_Multi(lin_spring, lin_damper, s_param=1.0)

# -------------------- Logging / Plotting --------------------
SAVE_PLOT = True
t0 = time.perf_counter()

t_log = []          # seconds
xyz_log = []        # (N,3) EE position
goal_log = []       # (N,3) current goal
tau_log = []        # (N,9) motor torques

waypoint_event_t = []
waypoint_event_goal = []

# -------------------- Control loop ------------------
def control_loop():
    global waypoint_index, stable_count

    # State from ROS2 topics (degrees -> radians)
    q_motor     = np.radians(controller.get_joint_positions())   # expected length 9
    q_motor_dot = np.radians(controller.get_joint_velocities())  # expected length 9

    goal = waypoints[waypoint_index]

    tau_vmc = vmc.multi_torques(q_motor, q_motor_dot, target_pos=goal)
    pose = FK_Multi(q_motor, vmc.s_param)   # [x,y,z,yaw,pitch,roll]

    # Check if motors have stopped moving
    if np.linalg.norm(q_motor_dot) < MOTOR_STOPPED_THRESHOLD:
        stable_count += 1
    else:
        stable_count = 0

    # Switch waypoints
    if stable_count >= STABLE_STEPS:
        stable_count = 0
        if waypoint_index == len(waypoints) - 1:
            print("Last waypoint reached")
        else:
            print(f"Reached waypoint {waypoint_index}: {waypoints[waypoint_index]}")
            print(f"Current position: {pose[:3]}\n")
            waypoint_index = (waypoint_index + 1) % len(waypoints)

        t_now = time.perf_counter() - t0
        next_goal = waypoints[waypoint_index]
        waypoint_event_t.append(t_now)
        waypoint_event_goal.append(next_goal.copy())

    # Safety clipping
    if np.any(np.abs(tau_vmc) > TAU_LIMIT):
        print("Torque limit exceeded! Clipping.")
    tau_vmc = np.clip(tau_vmc, -TAU_LIMIT, TAU_LIMIT)

    # Log
    t_now = time.perf_counter() - t0
    xyz = np.asarray(pose[:3], dtype=float)

    t_log.append(t_now)
    xyz_log.append(xyz)
    goal_log.append(goal.copy())
    tau_log.append(np.asarray(tau_vmc, dtype=float))

    # Publish
    controller.publish_torques(tau_vmc)

controller.create_timer(1.0 / CONTROL_FREQUENCY, control_loop)

# -------------------- Run + Save Plot on exit --------------------
try:
    rclpy.spin(controller)

except KeyboardInterrupt:
    pass

except BaseException as e:
    print(f"[WARN] Exception while spinning: {e}")

finally:
    try:
        controller.destroy_node()
    except Exception as e:
        print(f"[WARN] destroy_node failed: {e}")

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"[WARN] rclpy.shutdown failed (continuing): {e}")

    if len(t_log) < 2:
        print("[WARN] Not enough data logged to plot.")
    elif SAVE_PLOT:
        t_arr    = np.asarray(t_log)
        xyz_arr  = np.asarray(xyz_log)     # (N,3)
        goal_arr = np.asarray(goal_log)    # (N,3)
        tau_arr  = np.asarray(tau_log)     # (N,9)

        fig, (ax1, ax2) = plt.subplots(
            2, 1, sharex=True, figsize=(12, 8),
            gridspec_kw={"height_ratios": [2, 1]}
        )
        fig.suptitle("Multi Segment VMC: Position/Goal and Motor Torques vs Time")

        # Top: xyz + goal
        ax1.plot(t_arr, xyz_arr[:, 0], label="x")
        ax1.plot(t_arr, xyz_arr[:, 1], label="y")
        ax1.plot(t_arr, xyz_arr[:, 2], label="z")
        ax1.plot(t_arr, goal_arr[:, 0], "--", label="goal x", alpha=0.6)
        ax1.plot(t_arr, goal_arr[:, 1], "--", label="goal y", alpha=0.6)
        ax1.plot(t_arr, goal_arr[:, 2], "--", label="goal z", alpha=0.6)

        for te in waypoint_event_t:
            ax1.axvline(te, color="k", alpha=0.15, linewidth=1)

        ax1.set_ylabel("Position [m]")
        ax1.grid(True, alpha=0.3)
        ax1.legend(ncol=3)

        # Bottom: torques (9 motors)
        for k in range(9):
            ax2.plot(t_arr, tau_arr[:, k], label=f"tau{k+1}")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Torque [N·m]")
        ax2.grid(True, alpha=0.3)
        ax2.legend(ncol=3, fontsize=8)

        plt.tight_layout()

        # Save under MultiVMC/plots/
        script_dir = os.path.dirname(os.path.abspath(__file__))  # MultiVMC/
        plots_dir = os.path.join(script_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(plots_dir, f"vmc_multi_task_{timestamp}.png")

        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] Saved plot to: {out_path}")
    else:
        print("[INFO] Plot saving skipped (SAVE_PLOT=False).")