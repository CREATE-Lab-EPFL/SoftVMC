"""
# TERMINAL 1
# cd ros2_dynamixel && colcon build && source install/setup.bash
# sudo echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# ros2 run dynamixel_interface dynamixel_node --ros-args -p baudrate:=1000000 -p motor_ids:=[11,12,13,14,15,16,17,18,19] -p control_mode:=torque

# TERMINAL 2
# export PYTHONPATH="${PYTHONPATH}:$(pwd)"
# python3 Modular_MultiVMC_MovingGoal_Sub.py

# TERMINAL 3 (ONLY AFTER running Modular_MultiVMC_MovingGoal_Sub.py)
# source /opt/ros/humble/setup.bash
# python3 Modular_MultiVMC_MovingGoal_Pub.py

# TERMINAL 4 
# python3 MultiGoHome.py --read (COPY OUTPUT TO MultiGoHome.py)
# python3 MultiGoHome.py 
"""
import numpy as np
import rclpy
import time
from datetime import datetime
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray

from MultiController import MultiController, CONTROL_FREQUENCY
from FK_Multi import FK_Multi
from MultiVMCTaskSpace import VMC_Multi_Modular
from Modular_MultiVMC_Config import VIRTUAL_COMPONENTS, DEFAULT_GOALS, TAU_LIMIT, ANGLE_LIMIT_RAD, SAVE_PLOT


class MultiVMCModularNode(MultiController):
    def __init__(self):
        super().__init__()

        # VMC model (sum of all virtual components)
        self.vmc = VMC_Multi_Modular(VIRTUAL_COMPONENTS)
        self.n_comp = len(VIRTUAL_COMPONENTS)

        # Goals: flattened [x1,y1,z1, x2,y2,z2, ...]
        self.goal_flat = DEFAULT_GOALS.reshape(-1).copy()

        # Safety
        self.TAU_LIMIT = TAU_LIMIT
        self.ANGLE_LIMIT_RAD = ANGLE_LIMIT_RAD

        # ROS subscription for goals
        self.goal_sub = self.create_subscription(
            Float64MultiArray, "/goal_pos_multi", self.goal_cb, 10
        )

        # Logging
        self.SAVE_PLOT = SAVE_PLOT
        self.t0 = time.perf_counter()
        self.t_log = []
        self.tau_log = []            # commanded torque (N,9)
        self.goal_log = []           # flattened goals (N,3*n_comp)
        self.comp_pos_logs = [[] for _ in range(self.n_comp)]  # each: list of [x,y,z]

        # Timer for control loop
        self.control_timer_ = self.create_timer(1.0 / CONTROL_FREQUENCY, self.control_loop)

        self.get_logger().info(
            f"Started modular subscriber with {self.n_comp} components, topic=/goal_pos_multi"
        )

    def goal_cb(self, msg: Float64MultiArray):
        data = np.asarray(msg.data, dtype=float).reshape(-1)  # expect flat array of length 3*n_comp
        self.goal_flat = data.copy()

    def control_loop(self):
        # State from ROS2 topics (degrees -> radians)
        q_motor = np.radians(self.get_joint_positions())
        q_motor_dot = np.radians(self.get_joint_velocities())

        # Modular resultant torque
        tau_vmc = self.vmc.multi_torques(
            q_motor=q_motor,
            q_motor_dot=q_motor_dot,
            target_pos=self.goal_flat
        )

        # Safety limits
        if np.any(np.abs(tau_vmc) > self.TAU_LIMIT):
            self.get_logger().warn("Torque limit exceeded! Clipping.")
        tau_cmd = np.clip(tau_vmc, -self.TAU_LIMIT, self.TAU_LIMIT)

        if np.any(np.abs(q_motor) >= self.ANGLE_LIMIT_RAD):
            self.get_logger().warn("Angle limit exceeded! Stopping motors.")
            tau_cmd = np.zeros_like(tau_cmd)

        # Log time/torque/goals
        t_now = time.perf_counter() - self.t0
        self.t_log.append(t_now)
        self.tau_log.append(np.asarray(tau_cmd, dtype=float))
        self.goal_log.append(self.goal_flat.copy())

        # Log actual position of each virtual component point
        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            pose_i = FK_Multi(q_motor, comp.s_param)      # [x,y,z,yaw,pitch,roll]
            self.comp_pos_logs[i].append(np.asarray(pose_i[:3], dtype=float))

        # Publish torques
        self.publish_torques(tau_cmd)

    def save_plot(self):
        if len(self.t_log) < 2:
            print("[WARN] Not enough data logged to plot.")
            return

        t_arr = np.asarray(self.t_log)
        tau_arr = np.asarray(self.tau_log)            # (N,9)
        goal_arr = np.asarray(self.goal_log)          # (N,3*n_comp)

        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(13, 9))
        ax1, ax2 = axes
        fig.suptitle("Multi Modular VMC: Component Positions/Goals + Motor Torques")

        # Top: plot each component's actual xyz and goal xyz
        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            pos_i = np.asarray(self.comp_pos_logs[i])      # (N,3)
            goal_i = goal_arr[:, 3*i:3*i+3]               # (N,3)

            name = comp.name if comp.name else f"comp{i+1}"
            ax1.plot(t_arr, pos_i[:, 0], label=f"{name} x")
            ax1.plot(t_arr, pos_i[:, 1], label=f"{name} y")
            ax1.plot(t_arr, pos_i[:, 2], label=f"{name} z")

            ax1.plot(t_arr, goal_i[:, 0], "--", alpha=0.6, label=f"{name} goal x")
            ax1.plot(t_arr, goal_i[:, 1], "--", alpha=0.6, label=f"{name} goal y")
            ax1.plot(t_arr, goal_i[:, 2], "--", alpha=0.6, label=f"{name} goal z")

        ax1.set_ylabel("Position [m]")
        ax1.grid(True, alpha=0.3)
        ax1.legend(ncol=4, fontsize=8)

        # Bottom: torques
        for k in range(9):
            ax2.plot(t_arr, tau_arr[:, k], label=f"tau{k+11}")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Torque [N·m]")
        ax2.grid(True, alpha=0.3)
        ax2.legend(ncol=3, fontsize=8)

        plt.tight_layout()

        # Save
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        plots_dir = os.path.join(script_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(plots_dir, f"vmc_multi_modular_{timestamp}.png")
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] Saved plot to: {out_path}")


def main():
    rclpy.init()
    node = MultiVMCModularNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"[WARN] destroy_node failed: {e}")

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[WARN] rclpy.shutdown failed: {e}")

        if node.SAVE_PLOT:
            node.save_plot()


if __name__ == "__main__":
    main()