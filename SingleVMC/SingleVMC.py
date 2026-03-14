"""
Task-space VMC example.

Virtual springs and dampers are placed at the end effector of the single segment soft robot in Cartesian space.
The resulting forces are mapped to motor torques via J^T. 

Usage (from SoftVMC root):
    # from terminal 1 in SingleVMC folder
    cd ros2_dynamixel && colcon build && source install/setup.bash
    sudo echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    ros2 run dynamixel_interface dynamixel_node --ros-args -p baudrate:=1000000 -p motor_ids:=[1,2,3] -p control_mode:=torque
    
    # terminal 2 in SingleVMC folder
    export PYTHONPATH="${PYTHONPATH}:$(pwd)/SingleVMC"
    python3 SingleVMC.py

    # terminal 3 in SingleVMC folder (run at the end to back to home/rest position accurately)
    python3 SingleGoHome.py --read  # to read current home position 
    python3 SingleGoHome.py         # to go to home position

    # To check ports:
    ls /dev/tty*
"""
import numpy as np
import rclpy
import time
from datetime import datetime
import matplotlib.pyplot as plt

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearDamper, LinearSpring, TanhSpring, TanhDamper, GaussianSpring

from SingleController import SingleController, CONTROL_FREQUENCY
from SingleVMCTaskSpace import VMC_Single
from FK_Single import FK_Single

rclpy.init()
controller = SingleController()

# ------------------ Define waypoints in Cartesian space  ------------------

# Positions in Cartesian space [x, y, z] (meters)
POS_SCALE = 3
q1_short = np.array([0.01*POS_SCALE,                  0.0,                             0.144])  # q1 shorter (tau1 neg), q2,q3 equaly longer (tau2,tau3 pos)
q2_short = np.array([-0.01*POS_SCALE*np.sin(np.pi/6), 0.01*POS_SCALE*np.cos(np.pi/6),  0.144])  # q2 shorter (tau2 neg), q1,q3 equally longer (tau1,tau3 pos)
q3_short = np.array([-0.01*POS_SCALE*np.sin(np.pi/6), -0.01*POS_SCALE*np.cos(np.pi/6), 0.144]) # q3 shorter (tau3 neg), q1,q2 equally longer (tau1,tau2 pos)
rest_pos = np.array([0.0, 0.0, 0.145])

# Waypoints to follow
waypoints      = [q1_short, q2_short, q3_short, rest_pos] # once waypoints completed we go to rest position 
waypoint_index = 0

# ------------------ Control Loop constants and behavior control variables ------------------

# Stability check for switching waypoints (switch when prev_pos = current_pos -> waypoint reached)
stable_count   = -50             # negative to allow some initial settling time before the first switch
STABLE_STEPS   = 50              # must be stable for this many steps before switching
MOTOR_STOPPED_THRESHOLD = 0.1    # [rad/s] if motor velocity below this, consider it stopped
TAU_LIMIT      = 0.1             # [N·m] Safety limit for torques 

# ------------------- VMC Springs and Dampers ------------------ 

# Springs 
lin_spring   = LinearSpring(400.0)       # spring const [N/m] 
tahn_spring  = TanhSpring(200.0, 1.5)    # spring const [N/m],  max force [N] 
gauss_spring = GaussianSpring(800, 0.03) # force amplitude [N], sigma [m]

# Dampers
lin_damper  = LinearDamper(1.0)      # damping const [N·s/m] 
tahn_damper = TanhDamper(1.0, 1.5)   # damping const [N·s/m], max force [N]

# vmc_ee = VMC_Single_EE(lin_spring, lin_damper) # VMC_EE DEPRICATED USE VMC WITH S PARAM = 1.0 instead 
vmc = VMC_Single(lin_spring, lin_damper, s_param=1.0) 

# -------------------- Logging / Plotting --------------------
SAVE_PLOT = True 

t0 = time.perf_counter()

t_log = []          # seconds
xyz_log = []        # (N,3) EE position
goal_log = []       # (N,3) current goal at that time
tau_log = []        # (N,3) motor torques

# Optional: store events when waypoint switches (time + new goal)
waypoint_event_t = []
waypoint_event_goal = []

# -------------------- Control loop ------------------

def control_loop():
    global waypoint_index, stable_count

    # State from ROS2 topics (degrees -> radians)
    q_motor     = np.radians(controller.get_joint_positions())
    q_motor_dot = np.radians(controller.get_joint_velocities())

    goal = waypoints[waypoint_index]
    tau_vmc  = vmc.single_torques(q_motor, q_motor_dot, target_pos=goal) 
    pose = FK_Single(q_motor, vmc.s_param) # [x,y,z,yaw,pitch,roll] 

    # Check if motors have stopped moving
    # print(np.linalg.norm(q_motor_dot))
    if np.linalg.norm(q_motor_dot) < MOTOR_STOPPED_THRESHOLD: 
        stable_count += 1
    else:
        stable_count = 0   # reset if still moving
    
    # Only switch waypoints after motors stopped moving 
    if (stable_count >= STABLE_STEPS):
        stable_count = 0
        if waypoint_index == len(waypoints) - 1:
            print("Last waypoint reached")
        else:
            print(f"Reached waypoint {waypoint_index}: "f"{waypoints[waypoint_index]}")
            print(f"Current position: {pose[:3]}\n")
            waypoint_index = (waypoint_index + 1) % len(waypoints)

        # Record the waypoint switch event (time + next goal)
        t_now = time.perf_counter() - t0
        next_goal = waypoints[waypoint_index]
        waypoint_event_t.append(t_now)
        waypoint_event_goal.append(next_goal.copy())

    # Limit torque to 0.1 N·m for safety
    if np.any(np.abs(tau_vmc) > TAU_LIMIT):
        print("Torque limit exceeded! Clipping.")
    tau_vmc = np.clip(tau_vmc, -TAU_LIMIT, TAU_LIMIT)

    # --- Log values for plotting ---
    t_now = time.perf_counter() - t0
    xyz = np.asarray(pose[:3], dtype=float)
    t_log.append(t_now)
    xyz_log.append(xyz)
    goal_log.append(goal.copy())
    tau_log.append(np.asarray(tau_vmc, dtype=float))

    # Publish torques
    controller.publish_torques(tau_vmc)

controller.create_timer(1.0 / CONTROL_FREQUENCY, control_loop)

# -------------------- Run + Save Plot on exit  --------------------

try:
    rclpy.spin(controller)

except KeyboardInterrupt:
    pass

except BaseException as e:
    print(f"[WARN] Exception while spinning: {e}")

finally:
    # 1) Stop ROS node cleanly
    try:
        controller.destroy_node()
    except Exception as e:
        print(f"[WARN] destroy_node failed: {e}")

    # 2) Shutdown ROS only if not already shutdown (don't block plotting on errors)
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"[WARN] rclpy.shutdown failed (continuing): {e}")

    # 3) Save plots
    if len(t_log) < 2:
        print("[WARN] Not enough data logged to plot.")
    elif SAVE_PLOT:
        t_arr    = np.asarray(t_log)
        xyz_arr  = np.asarray(xyz_log)     # (N,3)
        goal_arr = np.asarray(goal_log)    # (N,3)
        tau_arr  = np.asarray(tau_log)     # (N,3)

        fig, (ax1, ax2) = plt.subplots(
            2, 1, sharex=True, figsize=(12, 8),
            gridspec_kw={"height_ratios": [2, 1]}
        )
        fig.suptitle("Single Segment VMC: Position/Goal and Motor Torques vs Time")

        # --- Top: xyz + goal ---
        ax1.plot(t_arr, xyz_arr[:, 0], label="x")
        ax1.plot(t_arr, xyz_arr[:, 1], label="y")
        ax1.plot(t_arr, xyz_arr[:, 2], label="z")
        ax1.plot(t_arr, goal_arr[:, 0], "--", label="goal x", alpha=0.6)
        ax1.plot(t_arr, goal_arr[:, 1], "--", label="goal y", alpha=0.6)
        ax1.plot(t_arr, goal_arr[:, 2], "--", label="goal z", alpha=0.6)

        # Mark waypoint switch times (vertical lines)
        for te in waypoint_event_t:
            ax1.axvline(te, color="k", alpha=0.15, linewidth=1)

        ax1.set_ylabel("Position [m]")
        ax1.grid(True, alpha=0.3)
        ax1.legend(ncol=3)

        # --- Bottom: torques ---
        ax2.plot(t_arr, tau_arr[:, 0], label="tau1")
        ax2.plot(t_arr, tau_arr[:, 1], label="tau2")
        ax2.plot(t_arr, tau_arr[:, 2], label="tau3")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Torque [N·m]")
        ax2.grid(True, alpha=0.3)
        ax2.legend(ncol=3)

        plt.tight_layout()

        # Save under SingleVMC/plots/
        script_dir = os.path.dirname(os.path.abspath(__file__))  # SingleVMC/
        plots_dir = os.path.join(script_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(plots_dir, f"vmc_single_task_{timestamp}.png")

        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] Saved plot to: {out_path}")
    else:
        print("[INFO] Plot saving skipped (SAVE_PLOT=False).")
