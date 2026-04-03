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

# ------------------ Define waypoints in Cartesian space ------------------
# Positions in Cartesian space [x, y, z] (meters)
POS_SCALE = 3

q1_short = np.array([0.01 * POS_SCALE,                   0.0,                              3 * 0.145 - 0.001])
q2_short = np.array([-0.01 * POS_SCALE * np.sin(np.pi/6), 0.01 * POS_SCALE * np.cos(np.pi/6), 3 * 0.145 - 0.001])
q3_short = np.array([-0.01 * POS_SCALE * np.sin(np.pi/6), -0.01 * POS_SCALE * np.cos(np.pi/6), 3 * 0.145 - 0.001])
rest_pos = np.array([0.0, 0.0, 3 * 0.145])

waypoints      = [q1_short, rest_pos]
waypoint_index = 0


# ------------------- VMC Springs and Dampers ------------------
lin_spring   = LinearSpring(300.0)
tanh_spring  = TanhSpring(200.0, 1.5)
gauss_spring = GaussianSpring(800, 0.03)

lin_damper   = LinearDamper(3.0)
tanh_damper  = TanhDamper(1.0, 1.5)

# Use s_param=1.0 (end of full backbone in your current FK/Jacobian setup)
vmc = VMC_Multi(lin_spring, lin_damper, s_param=1.0)

# -------------------- Single-pass plotting --------------------
SAVE_PLOT = True
SHOW_PLOT = True  # set False if running headless

if __name__ == "__main__":

    q_motor     = np.radians([0.0] * 9)   # expected length 9
    q_motor_dot = np.radians([0.0] * 9)   # expected length 9

    goal = waypoints[waypoint_index]

    # First (single-pass) torque computation
    tau_vmc = vmc.multi_torques(q_motor, q_motor_dot, target_pos=goal)
    pose = FK_Multi(q_motor, vmc.s_param)   # [x,y,z,yaw,pitch,roll]

    print(f"VMC torques: {tau_vmc}")
    print(f"FK pose: {pose[:3]} (target: {goal})")

    # ---------- Figure ----------
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10, 7),
        gridspec_kw={"height_ratios": [1, 2]}
    )
    fig.suptitle("One-Pass VMC Result: Expected Position vs First Computed Torque")

    # Top: expected vs current EE position (x,y,z) as grouped bars
    coords = ["x", "y", "z"]
    expected = np.asarray(goal, dtype=float)
    actual = np.asarray(pose[:3], dtype=float)

    x = np.arange(len(coords))
    w = 0.35
    ax1.bar(x - w/2, expected, width=w, label="expected (goal)", alpha=0.8)
    ax1.bar(x + w/2, actual,   width=w, label="actual (FK pose)", alpha=0.8)

    ax1.set_xticks(x)
    ax1.set_xticklabels(coords)
    ax1.set_ylabel("Position [m]")
    ax1.grid(True, axis="y", alpha=0.3)
    ax1.legend()

    # Bottom: first computed motor torques as bar chart
    tau = np.asarray(tau_vmc, dtype=float)
    motor_labels = [f"m{k+11}" for k in range(len(tau))]  # keeps your motor naming style
    ax2.bar(np.arange(len(tau)), tau, alpha=0.85)

    ax2.set_xticks(np.arange(len(tau)))
    ax2.set_xticklabels(motor_labels)
    ax2.set_ylabel("Torque [N·m]")
    ax2.set_xlabel("Motor")
    ax2.grid(True, axis="y", alpha=0.3)

    plt.tight_layout()

    # Save under MultiVMC/plots/
    script_dir = os.path.dirname(os.path.abspath(__file__))  # MultiVMC/
    plots_dir = os.path.join(script_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(plots_dir, f"one_pass_vmc{timestamp}.png")

    if SAVE_PLOT:
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        print(f"[OK] Saved plot to: {out_path}")

    if SHOW_PLOT:
        plt.show()
    else:
        plt.close(fig)