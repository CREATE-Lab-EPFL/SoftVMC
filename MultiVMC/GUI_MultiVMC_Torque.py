"""
vmc_torque_node.py
──────────────────
Standalone torque computation back-end.

Subscribes:
    /VC_Param          Float64MultiArray  [k1,..,kN,  c1,..,cN]
    /VC_ref_pos        Float64MultiArray  [x1,y1,z1, ..., xN,yN,zN]
    /joint_positions   Float64MultiArray  [deg] x 9
    /joint_velocities  Float64MultiArray  [deg/s] x 9

Publishes:
    /goal_torque       Float64MultiArray  [N·m] x 9  (robot hardware)
    /VMC_torque        Float64MultiArray  [N·m] x 9  (GUI / logging)

Run:
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"
    python3 vmc_torque_node.py
"""

import copy
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from MultiController import CONTROL_FREQUENCY
from MultiVMCTaskSpace import VMC_Multi_Modular
from MultiGravityComp import MultiGravComp
from GUI_MultiVMC_Config import (
    VIRTUAL_COMPONENTS,
    DEFAULT_GOALS,
    DEFAULT_START,
    N_VC,
    TAU_LIMIT,
    SPRING_PARAM_NAMES,
    DAMPER_PARAM_NAMES,
)


class VMCTorqueNode(Node):
    """
    Subscribes to parameter updates and reference positions from the GUI,
    then runs the VMC control loop at CONTROL_FREQUENCY Hz.

    The node keeps a *deep copy* of VIRTUAL_COMPONENTS so that parameter
    updates do not mutate the config module's originals.
    """

    def __init__(self):
        super().__init__("vmc_torque_node")

        # ── Deep-copy components so we can mutate spring/damper params ────────
        self._components = copy.deepcopy(VIRTUAL_COMPONENTS)
        self._rebuild_vmc()

        self._grav_comp = MultiGravComp()

        # ── Robot state ───────────────────────────────────────────────────────
        self._q      = np.zeros(9, dtype=float)   # rad
        self._q_dot  = np.zeros(9, dtype=float)   # rad/s
        self._goal   = DEFAULT_GOALS.reshape(-1).copy()   # flattened (3N,)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, "/VC_Param",          self._cb_param,    10)
        self.create_subscription(
            Float64MultiArray, "/VC_ref_pos",        self._cb_refpos,   10)
        self.create_subscription(
            Float64MultiArray, "/joint_positions",   self._cb_pos,      10)
        self.create_subscription(
            Float64MultiArray, "/joint_velocities",  self._cb_vel,      10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_hw  = self.create_publisher(Float64MultiArray, "/goal_torque",  10)
        self._pub_vmc = self.create_publisher(Float64MultiArray, "/VMC_torque",   10)

        # ── Control timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / CONTROL_FREQUENCY, self._control_loop)

        self.get_logger().info(
            f"VMCTorqueNode started — {N_VC} virtual component(s), "
            f"control @ {CONTROL_FREQUENCY} Hz"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rebuild_vmc(self):
        """(Re-)create VMC_Multi_Modular with current component list."""
        self._vmc = VMC_Multi_Modular(self._components)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_param(self, msg: Float64MultiArray):
        """
        Message layout: [k1, k2, …, kN,  c1, c2, …, cN]
        Updates the spring constant and damper constant of each component.
        """
        data = np.asarray(msg.data, dtype=float)
        expected = 2 * N_VC
        if len(data) < expected:
            self.get_logger().warn(
                f"/VC_Param: expected {expected} values, got {len(data)} — ignoring."
            )
            return

        k_vals = data[:N_VC]
        c_vals = data[N_VC: 2 * N_VC]

        for i, comp in enumerate(self._components):
            old_k = getattr(comp.spring, SPRING_PARAM_NAMES[i], None)
            old_c = getattr(comp.damper, DAMPER_PARAM_NAMES[i], None)
            setattr(comp.spring, SPRING_PARAM_NAMES[i], float(k_vals[i]))
            setattr(comp.damper, DAMPER_PARAM_NAMES[i], float(c_vals[i]))
            if old_k != k_vals[i] or old_c != c_vals[i]:
                self.get_logger().info(
                    f"  VC[{i}] {SPRING_PARAM_NAMES[i]}={k_vals[i]:.3f}  "
                    f"{DAMPER_PARAM_NAMES[i]}={c_vals[i]:.3f}"
                )

        # Rebuild VMC so it picks up the mutated component objects.
        # (Some VMC implementations cache Jacobians at construction time;
        #  if yours does not, you can skip the rebuild.)
        self._rebuild_vmc()

    def _cb_refpos(self, msg: Float64MultiArray):
        """Message layout: [x1,y1,z1, …, xN,yN,zN]"""
        data = np.asarray(msg.data, dtype=float)
        if len(data) == 3 * N_VC:
            self._goal = data.copy()
        else:
            self.get_logger().warn(
                f"/VC_ref_pos: expected {3 * N_VC} values, got {len(data)} — ignoring."
            )

    def _cb_pos(self, msg: Float64MultiArray):
        self._q = np.radians(np.asarray(msg.data, dtype=float))

    def _cb_vel(self, msg: Float64MultiArray):
        self._q_dot = np.radians(np.asarray(msg.data, dtype=float))

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        tau_vmc  = self._vmc.multi_torques(
            q_motor     = self._q,
            q_motor_dot = self._q_dot,
            target_pos  = self._goal,
        )
        tau_grav = self._grav_comp.gravity_compensation(self._q)
        tau_raw  = tau_vmc + tau_grav

        if np.any(np.abs(tau_raw) > TAU_LIMIT):
            self.get_logger().warn("Torque limit exceeded — clipping.")
        tau_cmd = np.clip(tau_raw, -TAU_LIMIT, TAU_LIMIT)

        self._publish(tau_cmd)

    def _publish(self, tau: np.ndarray):
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_hw.publish(msg)
        self._pub_vmc.publish(msg)


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = VMCTorqueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"[WARN] destroy_node: {e}")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[WARN] rclpy.shutdown: {e}")


if __name__ == "__main__":
    main()
