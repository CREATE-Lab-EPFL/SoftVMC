"""
GUI_MultiVMC_Torque.py
──────────────────────
Standalone torque computation back-end.

Subscribes:
    /VC_Param          Float64MultiArray  [k1,..,kN,  c1,..,cN]
    /VC_ref_pos        Float64MultiArray  [x1,y1,z1, ..., xN,yN,zN]
    /joint_positions   Float64MultiArray  [deg] x 9
    /joint_velocities  Float64MultiArray  [deg/s] x 9

Publishes:
    /goal_torque       Float64MultiArray  [N·m] x 9  (robot hardware + GUI)


TERMINAL 1
bash start_arm.sh   # homes the arm, sets USB latency, starts dynamixel_node

TERMINAL 2 (run FROM the experiment directory, e.g. LinearSpring/)
python3 ../VMC/GUI_MultiVMC_Noisy_Torque.py

TERMINAL 3 (also from the experiment directory)
python3 ../VMC/GUI_MultiVMC.py

TERMINAL 4
python3 ../VMC/MultiGoHome.py --read   (copy the printed values into MultiGoHome.py)
python3 ../VMC/MultiGoHome.py
"""

import sys, os
# Prepend the caller's cwd (not this script's own directory) so that running
# `python3 ../VMC/GUI_MultiVMC_Noisy_Torque.py` from inside an experiment folder picks up
# THAT folder's GUI_MultiVMC_Config.py, not VMC/GUI_MultiVMC_Config.py.
_CWD = os.getcwd()
sys.path.insert(0, _CWD)
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KINE = os.path.join(_ROOT, 'Kinematics')
for _p in [_ROOT, _KINE]:
    if _p not in sys.path: sys.path.insert(0, _p)

import argparse
import copy
import importlib
import time
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from MultiController import CONTROL_FREQUENCY
from MultiVMCTaskSpace import VMC_Multi_Modular  # multi_torques_and_forces used below
from MultiGravityComp import MultiGravComp
from FK_Multi import FK_Multi
from VMC_utils.GravityCompensation import GRAVITY_DIR_CHOICES, rotation_for_gravity_dir

_CONFIG_BASE = "GUI_MultiVMC_Config"


def _load_config():
    """
    Import the experiment config from the working directory.

    Default is the plain GUI_MultiVMC_Config, so a folder holding a single
    config needs nothing. `--config button` loads GUI_MultiVMC_Config_button
    instead: that is how a folder with several variants picks one, without
    copying a file into place first.
    """
    name, argv = _CONFIG_BASE, sys.argv[1:]
    for i, a in enumerate(argv):
        if a == "--config" and i + 1 < len(argv):
            name = f"{_CONFIG_BASE}_{argv[i + 1]}"
        elif a.startswith("--config="):
            name = f"{_CONFIG_BASE}_{a.split('=', 1)[1]}"
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError:
        variants = sorted(f[len(_CONFIG_BASE) + 1:-3] for f in os.listdir(_CWD)
                          if f.startswith(_CONFIG_BASE + "_") and f.endswith(".py"))
        sys.exit(f"[FATAL] no {name}.py in {_CWD}"
                 + (f" -- try --config {{{','.join(variants)}}}" if variants else ""))


_cfg = _load_config()

VIRTUAL_COMPONENTS   = _cfg.VIRTUAL_COMPONENTS
DEFAULT_START        = _cfg.DEFAULT_START
N_VC                 = _cfg.N_VC
TAU_LIMIT            = _cfg.TAU_LIMIT
SPRING_PARAM_NAMES   = _cfg.SPRING_PARAM_NAMES
DAMPER_PARAM_NAMES   = _cfg.DAMPER_PARAM_NAMES
SAVE_PLOT            = _cfg.SAVE_PLOT
SAVE_DATA            = _cfg.SAVE_DATA
PLOT_Z               = _cfg.PLOT_Z
PLOT_FORCE           = _cfg.PLOT_FORCE
PRETENSION_WEIGHTS   = _cfg.PRETENSION_WEIGHTS
PRETENSION_THRESHOLD = _cfg.PRETENSION_THRESHOLD
PRETENSION_ENABLED   = _cfg.PRETENSION_ENABLED

# True = real data already collected for this config -- never let a rerun
# (demo, sanity check, accidental restart) overwrite it. Configs that don't
# define it (e.g. connectivity-test folders) behave as before: False.
COLLECTED_DATA = getattr(_cfg, "COLLECTED_DATA", False)

# Direction gravity pulls in the ROBOT frame. "-z" (the default) is the arm
# standing straight up; a config for a rotated mount overrides it, e.g. the arm
# laid on its side pulls along "-y".
GRAVITY_DIR = getattr(_cfg, "GRAVITY_DIR", "-z")

# ── Perturbance config ─────────────────────────────────────────────────────────
PERTURB_ENABLED      = True               # master switch
PERTURB_STD          = 0.18                # [N·m] std of additive Gaussian noise (mean=0)
PERTURB_MOTORS       = list(range(9))     # motor indices to perturb (0-8, all by default)
PERIODIC_NOISE_TICKS = 300                # period: generate a new noise sample every N ticks
SAME_NOISE_TICKS     = 10                 # hold each sample constant for this many ticks (≤ PERIODIC_NOISE_TICKS)
NOISE_LIMIT          = TAU_LIMIT/2.0          # clip noise magnitude per motor
PERTURB_SEED         = 0                  # int for reproducibility, or None for system entropy
# ──────────────────────────────────────────────────────────────────────────────


class VMCTorqueNode(Node):
    """
    Subscribes to parameter updates and reference positions from the GUI,
    then runs the VMC control loop at CONTROL_FREQUENCY Hz.

    The node keeps a *deep copy* of VIRTUAL_COMPONENTS so that parameter
    updates do not mutate the config module's originals.
    """

    def __init__(self, gravity_dir=GRAVITY_DIR):
        super().__init__("vmc_torque_node")

        # ── Deep-copy components so we can mutate spring/damper params ────────
        self._components = copy.deepcopy(VIRTUAL_COMPONENTS)
        self._rebuild_vmc()

        self._gravity_dir = gravity_dir
        self._grav_comp = MultiGravComp(
            R_world_to_system=rotation_for_gravity_dir(gravity_dir)
        )
        self.get_logger().info(f"config: {_cfg.__name__}")
        self.get_logger().info(f"gravity compensation: gravity pulls along "
                               f"{gravity_dir} in the robot frame")

        # ── Robot state ───────────────────────────────────────────────────────
        self._q      = np.zeros(9, dtype=float)   # rad
        self._q_dot  = np.zeros(9, dtype=float)   # rad/s
        self._goal   = DEFAULT_START.reshape(-1).copy()   # flattened (3N,)

        # ── Logging ───────────────────────────────────────────────────────────
        self._t0             = None   # set on first /VC_ref_pos message
        self._t_log          = []
        self._tau_log        = []
        self._tau_grav_log   = []
        self._goal_log       = []
        self._comp_pos_logs  = [[] for _ in range(N_VC)]
        self._force_shear    = 0.0   # latest value from /force_shear
        self._force_log      = []    # sampled at control-loop rate
        self._f_virtual_log  = []    # (3,) task-space virtual force, sampled at control-loop rate
        self._k_log          = []    # (N_VC,) spring param per component, sampled at control-loop rate
        self._q_log          = []    # (9,) joint positions [rad], sampled at control-loop rate
        self._q_dot_log      = []    # (9,) joint velocities [rad/s], sampled at control-loop rate
        self._perturb_log    = []    # (9,) additive Gaussian noise applied per motor [N·m]
        self._tick_count     = 0     # control-loop tick counter (for periodic noise)
        self._current_noise  = np.zeros(9, dtype=float)  # held constant for SAME_NOISE_TICKS
        self._rng            = np.random.default_rng(PERTURB_SEED)  # reproducible noise

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, "/VC_Param",          self._cb_param,       10)
        self.create_subscription(
            Float64MultiArray, "/VC_ref_pos",        self._cb_refpos,      10)
        self.create_subscription(
            Float64MultiArray, "/joint_positions",   self._cb_pos,         10)
        self.create_subscription(
            Float64MultiArray, "/joint_velocities",  self._cb_vel,         10)
        self.create_subscription(
            Float64,           "/force_shear",       self._cb_force_shear, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_hw     = self.create_publisher(Float64MultiArray, "/goal_torque",     10)
        self._pub_vforce = self.create_publisher(Float64MultiArray, "/virtual_forces",  10)

        # ── Control timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / CONTROL_FREQUENCY, self._control_loop)

        self.get_logger().info(
            f"VMCTorqueNode started — {N_VC} virtual component(s), "
            f"control @ {CONTROL_FREQUENCY} Hz"
        )
        if PERTURB_ENABLED:
            self.get_logger().info(
                f"Perturbance ON — N(0, {PERTURB_STD:.4f}) N·m "
                f"on motors {PERTURB_MOTORS}, "
                f"period={PERIODIC_NOISE_TICKS} tick(s), "
                f"pulse width={SAME_NOISE_TICKS} tick(s), "
                f"seed={PERTURB_SEED}"
            )
        else:
            self.get_logger().info("Perturbance OFF")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rebuild_vmc(self):
        """(Re-)create VMC_Multi_Modular with current component list."""
        self._vmc = VMC_Multi_Modular(self._components)

    def _pretension_torque(self, q_motor: np.ndarray) -> np.ndarray:
        """tau_pretension[i] = W[i] * q[i]  if q[i] > threshold, else 0."""
        tau = np.where(q_motor > PRETENSION_THRESHOLD,
                       PRETENSION_WEIGHTS * q_motor,
                       0.0)
        return tau

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

        # No rebuild needed: VMC_Multi_Modular holds references to the same
        # VirtualComponent objects, so mutated spring/damper attrs are visible
        # immediately. J_funcs only depend on s_param (unchanged).

    def _cb_refpos(self, msg: Float64MultiArray):
        """Message layout: [x1,y1,z1, …, xN,yN,zN]"""
        data = np.asarray(msg.data, dtype=float)
        if len(data) == 3 * N_VC:
            if self._t0 is None:
                self._t0 = time.perf_counter()
            self._goal = data.copy()
        else:
            self.get_logger().warn(
                f"/VC_ref_pos: expected {3 * N_VC} values, got {len(data)} — ignoring."
            )

    def _cb_pos(self, msg: Float64MultiArray):
        self._q = np.radians(np.asarray(msg.data, dtype=float))

    def _cb_vel(self, msg: Float64MultiArray):
        self._q_dot = np.radians(np.asarray(msg.data, dtype=float))

    def _cb_force_shear(self, msg: Float64):
        self._force_shear = float(msg.data)

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        tau_vmc, f_virtual = self._vmc.multi_torques_and_forces(
            q_motor     = self._q,
            q_motor_dot = self._q_dot,
            target_pos  = self._goal,
        )
        tau_grav        = self._grav_comp.gravity_compensation(self._q)
        tau_pretension  = self._pretension_torque(self._q)
        if not PRETENSION_ENABLED:
            tau_pretension = np.zeros_like(tau_pretension)
        tau_raw         = tau_vmc + tau_grav - tau_pretension

        if np.any(np.abs(tau_raw) > TAU_LIMIT):
            self.get_logger().warn("Torque limit exceeded — clipping.")
        tau_cmd = np.clip(tau_raw, -TAU_LIMIT, TAU_LIMIT)

        # ── Additive perturbance (simulated external disturbance) ─────────────
        # Pulse pattern: every PERIODIC_NOISE_TICKS we draw a fresh noise vector
        # and hold it constant for SAME_NOISE_TICKS, zero in between.
        noise = np.zeros(9, dtype=float)
        if PERTURB_ENABLED:
            phase = self._tick_count % PERIODIC_NOISE_TICKS
            if phase == 0:
                new_noise = np.zeros(9, dtype=float)
                new_noise[PERTURB_MOTORS] = self._rng.normal(
                    0.0, PERTURB_STD, len(PERTURB_MOTORS))
                self._current_noise = np.clip(new_noise, -NOISE_LIMIT, NOISE_LIMIT)
                print(f"[PERTURB] new pulse @tick {self._tick_count}: {self._current_noise}")
            if phase < SAME_NOISE_TICKS:
                noise = self._current_noise

        self._tick_count += 1
        if np.any(np.abs(tau_cmd + noise) > TAU_LIMIT):
            self.get_logger().warn(
                f"Post-perturbation torque limit exceeded — clipping. "
                f"tau_cmd + noise = {tau_cmd + noise}"
            )
        tau_publish = np.clip(tau_cmd + noise, -TAU_LIMIT, TAU_LIMIT)
        # ─────────────────────────────────────────────────────────────────────

        self._publish(tau_publish)

        msg_vf = Float64MultiArray()
        msg_vf.data = f_virtual.tolist()
        self._pub_vforce.publish(msg_vf)

        # LOGS FOR PLOTTING:
        if self._t0 is None:
            return
        t_now = time.perf_counter() - self._t0
        self._t_log.append(t_now)
        self._tau_log.append(tau_cmd.copy())
        self._tau_grav_log.append(tau_grav.copy())
        self._goal_log.append(self._goal.copy())
        self._force_log.append(self._force_shear)
        self._f_virtual_log.append(f_virtual.copy())
        self._k_log.append([
            getattr(comp.spring, SPRING_PARAM_NAMES[i])
            for i, comp in enumerate(self._components)
        ])
        self._q_log.append(self._q.copy())
        self._q_dot_log.append(self._q_dot.copy())
        self._perturb_log.append(noise.copy())
        for i, comp in enumerate(self._components):
            pose_i = FK_Multi(self._q, float(comp.s_param))
            self._comp_pos_logs[i].append(np.asarray(pose_i[:3], dtype=float))

    def _publish(self, tau: np.ndarray):
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_hw.publish(msg)

    # ── Plot / save ───────────────────────────────────────────────────────────

    def save_plot(self):
        if len(self._t_log) < 2:
            print("[WARN] Not enough data logged to plot.")
            return

        lengths = ([len(self._t_log), len(self._tau_log),
                    len(self._tau_grav_log), len(self._goal_log),
                    len(self._force_log), len(self._f_virtual_log),
                    len(self._k_log), len(self._q_log), len(self._q_dot_log),
                    len(self._perturb_log)]
                   + [len(self._comp_pos_logs[i]) for i in range(N_VC)])
        n = min(lengths)
        if n < 2:
            print("[WARN] Not enough synchronized data to plot.")
            return

        t_arr          = np.asarray(self._t_log[:n])
        tau_arr        = np.asarray(self._tau_log[:n])         # (n, 9)
        tau_grav_arr   = np.asarray(self._tau_grav_log[:n])    # (n, 9)
        goal_arr       = np.asarray(self._goal_log[:n])        # (n, 3*N_VC)
        force_arr      = np.asarray(self._force_log[:n])       # (n,)
        f_virtual_arr  = np.asarray(self._f_virtual_log[:n])   # (n, 3)
        k_arr          = np.asarray(self._k_log[:n])            # (n, N_VC)
        q_arr          = np.asarray(self._q_log[:n])            # (n, 9)
        q_dot_arr      = np.asarray(self._q_dot_log[:n])        # (n, 9)
        perturb_arr    = np.asarray(self._perturb_log[:n])      # (n, 9)
        comp_pos_arrs  = [
            np.asarray(self._comp_pos_logs[i][:n]) for i in range(N_VC)
        ]                                                       # each (n, 3)

        n_pos_axes = 3 if PLOT_Z else 2
        n_rows = n_pos_axes + (1 if PLOT_FORCE else 0) + 3   # +1 virtual forces, +1 spring params, +1 torques
        fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(13, 3.2 * n_rows))
        fig.suptitle("GUI Multi VMC: Position Components + Motor Torques", y=0.97)

        # ── Metadata box ──────────────────────────────────────────────────────
        def _comp_desc(comp) -> str:
            def fmt(obj):
                d = getattr(obj, "__dict__", {})
                return ", ".join(f"{k}={v}" for k, v in d.items()) if d else ""
            sname = type(comp.spring).__name__
            dname = type(comp.damper).__name__
            sp = fmt(comp.spring)
            dp = fmt(comp.damper)
            return (f"s={comp.s_param:.3f} | "
                    f"spring={sname}({sp}) | damper={dname}({dp})")

        meta_lines = []
        for i, comp in enumerate(self._components):
            cname = comp.name or f"VC{i+1}"
            meta_lines.append(f"{cname}: {_comp_desc(comp)}")
        meta_lines.append(
            f"control_freq={CONTROL_FREQUENCY} Hz | tau_limit={TAU_LIMIT} N·m"
        )
        fig.subplots_adjust(top=0.82)
        fig.text(
            0.5, 0.92, "\n".join(meta_lines),
            ha="center", va="top", fontsize=8.5, family="monospace",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9, edgecolor="gray"),
        )

        # ── Position axes ─────────────────────────────────────────────────────
        ax_x      = axes[0]
        ax_y      = axes[1]
        ax_z      = axes[2] if PLOT_Z else None
        ax_force  = axes[2 + (1 if PLOT_Z else 0)] if PLOT_FORCE else None
        ax_vforce = axes[n_pos_axes + (1 if PLOT_FORCE else 0)]
        ax_k      = axes[n_pos_axes + (1 if PLOT_FORCE else 0) + 1]
        ax_tau    = axes[-1]

        goal_cmap = plt.get_cmap("tab10", N_VC)
        for i, comp in enumerate(self._components):
            pos  = comp_pos_arrs[i]          # (n, 3) actual FK position
            goal = goal_arr[:, 3*i: 3*i+3]  # (n, 3) reference
            name = comp.name or f"VC{i+1}"
            c    = goal_cmap(i)

            ax_x.plot(t_arr, pos[:, 0],  color=c, linewidth=2.0, label=f"{name} pos")
            ax_x.plot(t_arr, goal[:, 0], "--", color=c, alpha=0.8, label=f"{name} goal")

            ax_y.plot(t_arr, pos[:, 1],  color=c, linewidth=2.0, label=f"{name} pos")
            ax_y.plot(t_arr, goal[:, 1], "--", color=c, alpha=0.8, label=f"{name} goal")

            if PLOT_Z:
                ax_z.plot(t_arr, pos[:, 2],  color=c, linewidth=2.0, label=f"{name} pos")
                ax_z.plot(t_arr, goal[:, 2], "--", color=c, alpha=0.8, label=f"{name} goal")

        for ax, label in [(ax_x, "X [m]"), (ax_y, "Y [m]")]:
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
            ax.legend(ncol=4, fontsize=8, loc="lower right")
        if PLOT_Z:
            ax_z.set_ylabel("Z [m]")
            ax_z.grid(True, alpha=0.3)
            ax_z.legend(ncol=4, fontsize=8, loc="lower right")

        if PLOT_FORCE:
            ax_force.plot(t_arr, force_arr, color="darkorange", linewidth=1.6,
                          label="shear force")
            ax_force.set_ylabel("F_shear [N]")
            ax_force.grid(True, alpha=0.3)
            ax_force.legend(fontsize=8, loc="lower right")

        # ── Virtual forces axis ───────────────────────────────────────────────
        vf_colors = ["tab:blue", "tab:orange", "tab:green"]
        for k, (lbl, c) in enumerate(zip(["Fx", "Fy", "Fz"], vf_colors)):
            ax_vforce.plot(t_arr, f_virtual_arr[:, k], color=c, linewidth=1.6, label=lbl)
        ax_vforce.set_ylabel("F_virtual [N]")
        ax_vforce.grid(True, alpha=0.3)
        ax_vforce.legend(ncol=3, fontsize=8, loc="lower right")

        # ── Spring param (k) axis ─────────────────────────────────────────────
        k_cmap = plt.get_cmap("tab10", N_VC)
        for i, comp in enumerate(self._components):
            name  = comp.name or f"VC{i+1}"
            pname = SPRING_PARAM_NAMES[i]
            ax_k.plot(t_arr, k_arr[:, i], color=k_cmap(i), linewidth=1.6,
                      label=f"{name} {pname}")
        ax_k.set_ylabel("Spring param")
        ax_k.grid(True, alpha=0.3)
        ax_k.legend(ncol=N_VC, fontsize=8, loc="lower right")

        # ── Torque axis ───────────────────────────────────────────────────────
        for k in range(9):
            label = f"tau{k+11}"
            ln, = ax_tau.plot(t_arr, tau_arr[:, k], "-", linewidth=1.6, label=label)
            ax_tau.plot(t_arr, tau_grav_arr[:, k], "--", linewidth=1.4, alpha=0.9,
                        color=ln.get_color(), label=f"{label}_grav")

        ax_tau.set_xlabel("Time [s]")
        ax_tau.set_ylabel("Torque [N·m]")
        ax_tau.grid(True, alpha=0.3)
        ax_tau.legend(ncol=3, fontsize=8, loc="lower right")

        # ── Save plot ─────────────────────────────────────────────────────────
        # Saved relative to the caller's cwd (the experiment directory), matching
        # where GUI_MultiVMC_Config.py was loaded from -- NOT this script's own
        # directory -- so output lands next to the experiment it belongs to.
        plots_dir = os.path.join(_CWD, "experiments")
        os.makedirs(plots_dir, exist_ok=True)
        stamp    = datetime.now().strftime("%d_%H%M")
        out_path = os.path.join(plots_dir, f"z_{stamp}.png")
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] Saved plot → {out_path}")

        # ── Save raw data (.npz) ──────────────────────────────────────────────
        if SAVE_DATA:
            # goal_arr is (n, 3*N_VC) → reshape to (n, N_VC, 3) for clean indexing
            goal_3d     = goal_arr.reshape(n, N_VC, 3)
            # comp_pos_arrs is list of N_VC × (n, 3) → stack to (n, N_VC, 3)
            comp_pos_3d = np.stack(comp_pos_arrs, axis=1)

            npz_path = os.path.join(plots_dir, f"z_{stamp}.npz")
            np.savez_compressed(
                npz_path,
                # ── time ──────────────────────────────────────────────────────
                t            = t_arr,          # (n,)        [s]
                # ── torques ───────────────────────────────────────────────────
                tau          = tau_arr,         # (n, 9)      [N·m]  commanded
                tau_grav     = tau_grav_arr,    # (n, 9)      [N·m]  grav comp
                # ── positions ─────────────────────────────────────────────────
                goal         = goal_3d,         # (n, N_VC, 3) [m]  reference
                comp_pos     = comp_pos_3d,     # (n, N_VC, 3) [m]  FK actual
                # ── force ─────────────────────────────────────────────────────
                force        = force_arr,       # (n,)        [N]   shear
                f_virtual    = f_virtual_arr,   # (n, 3)      [N]   task-space virtual force
                k_params     = k_arr,           # (n, N_VC)   spring param per component over time
                q_motor      = q_arr,           # (n, 9)      [rad] joint positions
                q_motor_dot  = q_dot_arr,       # (n, 9)      [rad/s] joint velocities
                perturb      = perturb_arr,     # (n, 9)      [N·m]  additive noise per motor
                # ── metadata ──────────────────────────────────────────────────
                comp_names   = np.array([c.name or f"VC{i+1}"
                                         for i, c in enumerate(self._components)]),
                comp_s_params= np.array([c.s_param for c in self._components]),
                control_freq = np.array([CONTROL_FREQUENCY]),   # [Hz]
                tau_limit    = np.array([TAU_LIMIT]),            # [N·m]
            )
            print(f"[OK] Saved data  → {npz_path}")


# ── Entry point ────────────────────────────────────────────────────────────────

def _glue_signed_values(argv, flags=("--gravity-dir",)):
    """
    argparse reads a leading "-" as another flag, so `--gravity-dir -y` would
    fail. Rewrite it to `--gravity-dir=-y` and let both spellings work.
    """
    out, i = [], 0
    while i < len(argv):
        if argv[i] in flags and i + 1 < len(argv):
            out.append(f"{argv[i]}={argv[i + 1]}")
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", metavar="NAME", default=None,
                   help=f"load GUI_MultiVMC_Config_NAME.py from this directory "
                        f"instead of GUI_MultiVMC_Config.py (loaded: {_cfg.__name__})")
    p.add_argument("--gravity-dir", default=GRAVITY_DIR, metavar="DIR",
                   choices=GRAVITY_DIR_CHOICES,
                   help="direction gravity pulls in the robot frame "
                        f"(default from this folder's config: {GRAVITY_DIR}). "
                        '"-z" is the arm standing straight up, "-y" the arm '
                        "rotated onto its side.")
    args, _ = p.parse_known_args(_glue_signed_values(sys.argv[1:]))  # ignore ROS args
    return args


def main():
    args = _parse_args()
    rclpy.init()
    node = VMCTorqueNode(gravity_dir=args.gravity_dir)
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

        if SAVE_PLOT and not COLLECTED_DATA:
            node.save_plot()
        elif COLLECTED_DATA:
            print("[INFO] COLLECTED_DATA=True -- not saving (data already collected; "
                  "set COLLECTED_DATA=False in GUI_MultiVMC_Config.py to record a new run).")


if __name__ == "__main__":
    main()
