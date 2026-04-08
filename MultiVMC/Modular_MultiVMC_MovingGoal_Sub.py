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
from Modular_MultiVMC_Config import DEFAULT_START, VIRTUAL_COMPONENTS, TAU_LIMIT, ANGLE_LIMIT_RAD, SAVE_PLOT, PLOT_Z


class MultiVMCModularNode(MultiController):
    def __init__(self):
        super().__init__() # takes ~7 seconds

        # VMC model (sum of all virtual components)
        self.vmc = VMC_Multi_Modular(VIRTUAL_COMPONENTS)
        self.n_comp = len(VIRTUAL_COMPONENTS)

        # Goals: flattened [x1,y1,z1, x2,y2,z2, ...]
        self.goal_flat = DEFAULT_START.reshape(-1).copy()

        # Safety
        self.TAU_LIMIT = TAU_LIMIT
        self.ANGLE_LIMIT_RAD = ANGLE_LIMIT_RAD

        # ROS subscription for goals (100 Hz)
        self.goal_sub = self.create_subscription(
            Float64MultiArray, "/goal_pos_multi", self.goal_cb, 10
        )

        # Logging
        self.SAVE_PLOT = SAVE_PLOT
        self.t0 = None
        self.t_log = []
        self.tau_log = []            # commanded torque (N,9)
        self.goal_log = []           # flattened goals (N,3*n_comp)
        self.comp_pos_logs = [[] for _ in range(self.n_comp)]  # each: list of [x,y,z]

        # Timer for control loop (330 Hz)
        self.control_timer_ = self.create_timer(1.0 / CONTROL_FREQUENCY, self.control_loop)

        self.get_logger().info(f"Started modular VMC subscriber, you can now run the goal publisher")

    def goal_cb(self, msg: Float64MultiArray):
        # start time on first received goal
        if self.t0 is None:
            self.t0 = time.perf_counter() 
        # data = [x1,y1,z1,...,xN,yN,zN]  where N = number of components = n_comp
        data = np.asarray(msg.data, dtype=float).reshape(-1)  
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

        # if np.any(np.abs(q_motor) >= self.ANGLE_LIMIT_RAD):
        #     self.get_logger().warn("Angle limit exceeded! Stopping motors.")
        #     tau_cmd = np.zeros_like(tau_cmd)

        # Log time/torque/goals
        if self.t0 is None:
            return  # wait until first goal arrives
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

        # ---- synchronize lengths to avoid occasional shutdown race mismatch ----
        lengths = [len(self.t_log), len(self.tau_log), len(self.goal_log)]
        lengths += [len(self.comp_pos_logs[i]) for i in range(self.n_comp)]
        n = min(lengths)

        if n < 2:
            print("[WARN] Not enough synchronized data logged to plot.")
            return

        t_arr = np.asarray(self.t_log[:n])
        tau_arr = np.asarray(self.tau_log[:n])                      # (n,9)
        goal_arr = np.asarray(self.goal_log[:n])                    # (n,3*n_comp)
        comp_pos_arrs = [np.asarray(self.comp_pos_logs[i][:n]) for i in range(self.n_comp)]  # each (n,3)

        t_arr = np.asarray(self.t_log)
        tau_arr = np.asarray(self.tau_log)            # (N,9)
        goal_arr = np.asarray(self.goal_log)          # (N,3*n_comp)

        # Position axes: x,y (+ optional z), plus torque axis at bottom
        n_pos_axes = 3 if PLOT_Z else 2
        n_rows = n_pos_axes + 1  # +1 torque plot
        fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(13, 3.2 * n_rows))
        fig.suptitle("Multi Modular VMC: Position Components + Motor Torques")

        # --- METADATA BOX ---
        def _component_desc(comp) -> str:
            spring = comp.spring
            damper = comp.damper

            def fmt_params(obj):
                if not hasattr(obj, "__dict__") or len(obj.__dict__) == 0:
                    return ""
                return ", ".join(f"{k}={v}" for k, v in obj.__dict__.items())

            sname = type(spring).__name__
            dname = type(damper).__name__
            sparams = fmt_params(spring)
            dparams = fmt_params(damper)

            spring_txt = f"{sname}({sparams})" if sparams else sname
            damper_txt = f"{dname}({dparams})" if dparams else dname
            return f"s={comp.s_param:.3f} | spring={spring_txt} | damper={damper_txt}"

        # ---- centered metadata under title ----
        meta_lines = []
        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            cname = comp.name if comp.name else f"seg{i+1}"
            meta_lines.append(f"{cname}: {_component_desc(comp)}")

        # wrap to avoid one super-long line
        # 1 component per line (simple + clean). You can combine if you prefer.
        meta_text = "\n".join(meta_lines)

        # Make room at top for title + metadata
        fig.subplots_adjust(top=0.82)

        # Title slightly higher
        fig.suptitle("Multi Modular VMC: Position Components + Motor Torques", y=0.97)

        # Centered metadata box under title
        fig.text(
            0.5, 0.92, meta_text,
            ha="center", va="top", fontsize=8.5, family="monospace",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9, edgecolor="gray")
        )

        # --- PLOTS ---
        ax_x = axes[0]
        ax_y = axes[1]
        ax_z = axes[2] if PLOT_Z else None
        ax_tau = axes[-1]

        # Position plots split by component axis
        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            pos_i = comp_pos_arrs[i]                                # (n,3)
            goal_i = goal_arr[:, 3*i:3*i+3]                        # (n,3)
            name = comp.name if comp.name else f"seg{i+1}"

            line_x, = ax_x.plot(t_arr, pos_i[:, 0], label=f"{name}")
            ax_x.plot(t_arr, goal_i[:, 0], "--", color=line_x.get_color(), alpha=0.8, label=f"goal_{name}")

            line_y, = ax_y.plot(t_arr, pos_i[:, 1], label=f"{name}")
            ax_y.plot(t_arr, goal_i[:, 1], "--", color=line_y.get_color(), alpha=0.8, label=f"goal_{name}")

            if PLOT_Z:
                line_z, = ax_z.plot(t_arr, pos_i[:, 2], label=f"{name}")
                ax_z.plot(t_arr, goal_i[:, 2], "--", color=line_z.get_color(), alpha=0.8, label=f"goal_{name}")
                
        ax_x.set_ylabel("X [m]")
        ax_x.grid(True, alpha=0.3)
        ax_x.legend(ncol=4, fontsize=8)

        ax_y.set_ylabel("Y [m]")
        ax_y.grid(True, alpha=0.3)
        ax_y.legend(ncol=4, fontsize=8)

        if PLOT_Z:
            ax_z.set_ylabel("Z [m]")
            ax_z.grid(True, alpha=0.3)
            ax_z.legend(ncol=4, fontsize=8)

        # Torque plot
        for k in range(9):
            ax_tau.plot(t_arr, tau_arr[:, k], label=f"tau{k+11}")
        ax_tau.set_xlabel("Time [s]")
        ax_tau.set_ylabel("Torque [N·m]")
        ax_tau.grid(True, alpha=0.3)
        ax_tau.legend(ncol=3, fontsize=8)

        plt.tight_layout()

        # Save
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        plots_dir = os.path.join(script_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%H%M%S")
        out_path = os.path.join(plots_dir, f"vmc_{timestamp}.png")
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