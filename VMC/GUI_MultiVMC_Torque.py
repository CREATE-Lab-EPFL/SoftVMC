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
python3 ../VMC/GUI_MultiVMC_Torque.py

TERMINAL 3 (also from the experiment directory)
python3 ../VMC/GUI_MultiVMC.py

TERMINAL 4
python3 ../VMC/MultiGoHome.py --read   (copy the printed values into MultiGoHome.py)
python3 ../VMC/MultiGoHome.py
"""

import sys, os
# Prepend the caller's cwd (not this script's own directory) so that running
# `python3 ../VMC/GUI_MultiVMC_Torque.py` from inside an experiment folder picks up
# THAT folder's GUI_MultiVMC_Config.py, not VMC/GUI_MultiVMC_Config.py.
_CWD = os.getcwd()
sys.path.insert(0, _CWD)
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KINE = os.path.join(_ROOT, 'Kinematics')
for _p in [_ROOT, _KINE]:
    if _p not in sys.path: sys.path.insert(0, _p)

import argparse
import copy
import json
import signal
import threading
import time
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

# Latched, so a driver starting later still learns which config is live.
CONFIG_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

from MultiController import CONTROL_FREQUENCY
from MultiVMCTaskSpace import VMC_Multi_Modular  # multi_torques_and_forces used below
from MultiGravityComp import MultiGravComp
from FK_Multi import FK_Multi
from VMC_utils.GravityCompensation import GRAVITY_DIR_CHOICES, rotation_for_gravity_dir
from VMC_utils.ConfigHelpers import (config_from_argv, condition_strength,
                                     condition_targets)

# --config NAME picks GUI_MultiVMC_Config_NAME.py from the launch directory;
# without it the plain GUI_MultiVMC_Config.py is used, as before.
_cfg = config_from_argv(directory=_CWD)

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
PRETENSION_ENABLED   = _cfg.PRETENSION_ENABLED

# True = real data already collected for this config -- never let a rerun
# (demo, sanity check, accidental restart) overwrite it. Configs that don't
# define it (e.g. connectivity-test folders) behave as before: False.
COLLECTED_DATA = getattr(_cfg, "COLLECTED_DATA", False)

# Subfolder of outputs/ that this experiment's named runs belong in.
OUTPUT_FOLDER = getattr(_cfg, "OUTPUT_FOLDER", None)

# Seconds between checkpoints while a run is going: the logs so far are written
# to <run>.autosave.npz and a status line is printed. 0 (the default) keeps the
# old behaviour -- nothing reaches disk until exit, and a kill -9 loses the run.
# Set it in a config to opt in; the write happens on its own thread so the
# control loop is not blocked by it.
AUTOSAVE_S = float(getattr(_cfg, "AUTOSAVE_S", 0.0))

# The torque node drives the arm from its first tick -- the goal is already at
# DEFAULT_START and a run's condition is already ramping -- but the GUI usually
# comes up seconds later. Those first seconds used to go unlogged, taking the
# take-up transient and the whole ramp with them. They are logged now, at
# negative t (t = 0 is still the first /VC_ref_pos), and trimmed to the most
# recent PRE_REFERENCE_S so a node left idling cannot grow without bound.
PRE_REFERENCE_S = float(getattr(_cfg, "PRE_REFERENCE_S", 60.0))

# Moments to mark with Ctrl+C in THIS terminal, in order: the first press marks
# the first of these, the last press ends the run as it always did. Without a
# mark nothing in the log says when the arm actually arrived, and the
# measurement window has to be guessed from the longest stretch nothing was
# touched -- which cannot tell "on the button" from "let go of the mouse".
# Empty (the default) leaves Ctrl+C meaning exactly what it meant before, so a
# folder that does not set this is untouched.
MARK_PROMPTS = tuple(getattr(_cfg, "MARK_PROMPTS", ()))

# Direction gravity pulls in the ROBOT frame. "-z" (the default) is the arm
# standing straight up; a config for a rotated mount overrides it, e.g. the arm
# laid on its side pulls along "-y".
GRAVITY_DIR = getattr(_cfg, "GRAVITY_DIR", "-z")


class VMCTorqueNode(Node):
    """
    Subscribes to parameter updates and reference positions from the GUI,
    then runs the VMC control loop at CONTROL_FREQUENCY Hz.

    The node keeps a *deep copy* of VIRTUAL_COMPONENTS so that parameter
    updates do not mutate the config module's originals.
    """

    def __init__(self, gravity_dir=GRAVITY_DIR, run_name=None):
        super().__init__("vmc_torque_node")

        # ── Deep-copy components so we can mutate spring/damper params ────────
        self._components = copy.deepcopy(VIRTUAL_COMPONENTS)

        # A named run can carry its own spring strength (GAUSS_CONDITIONS), so
        # the run name alone defines the condition. Configs without them are
        # untouched.
        # A named run carries its own spring strength (GAUSS_CONDITIONS). It is
        # ramped in from 0 over GAUSS_RAMP_S rather than applied at once: the
        # obstacle centre is already placed, so a field switched on at full
        # strength shoves the arm off it within a single tick.
        self._cond_targets = condition_targets(_cfg)
        self._ramp_target  = condition_strength(_cfg, run_name)
        self._ramp_s       = float(getattr(_cfg, "GAUSS_RAMP_S", 0.0))
        self._ramp_t0      = None
        self._ramp_done    = self._ramp_target is None
        self.get_logger().info("building Jacobians (sympy -- takes a while, Ctrl+C to abort)")
        _t_build = time.perf_counter()

        if not self._ramp_done:
            ramping = self._ramp_s > 0.0
            self._set_condition(0.0 if ramping else self._ramp_target)
            self._ramp_done = not ramping
            self.get_logger().info(
                f"condition {run_name}: {', '.join(self._cond_targets)} -> "
                f"strength {self._ramp_target}"
                + (f", ramped in over {self._ramp_s:.0f} s" if ramping else ""))
        self._rebuild_vmc()

        self._run_name    = run_name
        self._gravity_dir = gravity_dir
        self._grav_comp = MultiGravComp(
            R_world_to_system=rotation_for_gravity_dir(gravity_dir)
        )
        self.get_logger().info(f"Jacobians ready in {time.perf_counter() - _t_build:.0f} s")
        self._pub_cfg = self.create_publisher(String, "/vmc_config", CONFIG_QOS)
        self._pub_cfg.publish(String(data=_cfg.__name__))
        self.get_logger().info(f"config: {_cfg.__name__}")
        if run_name:
            self.get_logger().info(
                f"run: {run_name} -- saving to {self._out_dir()}/{run_name}.{{png,npz}}, "
                f"overwriting any previous {run_name}")
        self.get_logger().info(f"gravity compensation: gravity pulls along "
                               f"{gravity_dir} in the robot frame")

        # ── Robot state ───────────────────────────────────────────────────────
        self._q      = np.zeros(9, dtype=float)   # rad
        self._q_dot  = np.zeros(9, dtype=float)   # rad/s
        self._goal   = DEFAULT_START.reshape(-1).copy()   # flattened (3N,)

        # ── Logging ───────────────────────────────────────────────────────────
        # Everything the control loop computes is kept, at control rate -- a
        # run is a few MB and it happens once. Only what is exactly derivable
        # offline is left out: the backbone at any s follows from q_motor, and
        # the summed virtual force from the per-component ones.
        self._t0             = None   # perf_counter of the first /VC_ref_pos message
        self._t_log          = []    # (1,) [s]  raw perf_counter; re-based on save
        self._t_wall_log     = []    # (1,) [s]  unix time, to line this up with other logs
        self._tau_log        = []    # (9,) [N.m] commanded, AFTER clipping
        self._tau_vmc_log    = []    # (9,) [N.m] virtual-model share, before clipping
        self._tau_grav_log   = []    # (9,) [N.m] gravity-compensation share
        self._tau_pre_log    = []    # (9,) [N.m] pretension share (subtracted)
        self._tau_comp_log   = []    # (N_VC, 9) [N.m] each element's share of tau_vmc
        self._goal_log       = []    # (3*N_VC,) [m] reference
        self._comp_pos_log   = []    # (N_VC, 3) [m] FK position of each attachment point
        self._comp_vel_log   = []    # (N_VC, 3) [m/s] its task-space velocity
        self._f_comp_log     = []    # (N_VC, 3) [N] each element's task-space force
        self._force_shear    = 0.0   # latest value from /force_shear
        self._force_stamp    = None  # when it arrived -- a flat channel may be a dead node
        self._force_log      = []    # (1,) sampled at control-loop rate
        self._force_age_log  = []    # (1,) [s] age of that reading when sampled
        self._k_log          = []    # (N_VC,) spring param per component over time
        self._c_log          = []    # (N_VC,) damper param per component over time
        self._q_log          = []    # (9,) [rad] joint positions
        self._q_dot_log      = []    # (9,) [rad/s] joint velocities

        # Operator marks: (perf_counter, unix time, label), re-based on save.
        self._marks = []
        self._pending_marks = []

        # Named once, at the start: an unnamed run is stamped with when it began.
        self._base = self._run_name or ("z_" + datetime.now().strftime("%d_%H%M"))
        if MARK_PROMPTS:
            self._install_mark_handler()

        self._stop_autosave = threading.Event()
        if AUTOSAVE_S > 0.0:
            threading.Thread(target=self._autosave_loop, daemon=True).start()
            self.get_logger().info(
                f"checkpointing every {AUTOSAVE_S:.0f} s to {self._base}.autosave.npz")

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

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_condition(self, value):
        """Set the condition's spring parameter on every component it applies to."""
        for i, c in enumerate(self._components):
            if c.name in self._cond_targets:
                setattr(c.spring, SPRING_PARAM_NAMES[i], float(value))

    def _rebuild_vmc(self):
        """(Re-)create VMC_Multi_Modular with current component list."""
        self._vmc = VMC_Multi_Modular(self._components)

    def _pretension_torque(self, q_motor: np.ndarray) -> np.ndarray:
        """
        tau_pretension[i] = W[i] * q[i] -- always on, continuous through zero.

        This used to switch on only above a 30 deg threshold, which put a step
        of W*threshold into the torque law exactly where a motor crossed it.
        At W = 0.03 that step was 0.0157 N.m, more than twice the whole VMC
        torque it was meant to sit underneath. Now it is a small continuous
        bias with nothing to chatter against.
        """
        return PRETENSION_WEIGHTS * q_motor

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

        if not self._ramp_done:
            self._ramp_done = True
            self.get_logger().info("/VC_Param received -- gauss ramp cancelled, sliders take over")

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
                # t = 0. Also the cue a scripted run waits on: the publisher is
                # up, so the arm is now being driven at the configured goal.
                self.get_logger().info("first reference received -- t = 0")
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
        self._force_stamp = time.perf_counter()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._ramp_done:
            if self._ramp_t0 is None:
                self._ramp_t0 = time.perf_counter()
            frac = min(1.0, (time.perf_counter() - self._ramp_t0) / self._ramp_s)
            self._set_condition(frac * self._ramp_target)
            if frac >= 1.0:
                self._ramp_done = True
                self.get_logger().info(f"gauss ramp complete -- strength {self._ramp_target}")

        # The per-component form: same torque, but it also hands back the FK
        # position, the task-space velocity and the force of every element, so
        # the logs below cost no second kinematics pass and record exactly the
        # pose this torque was computed from.
        state           = self._vmc.multi_torques_forces_states(
            q_motor     = self._q,
            q_motor_dot = self._q_dot,
            target_pos  = self._goal,
        )
        tau_vmc         = state["tau"]
        f_virtual       = state["F_comp"].sum(axis=0)
        tau_grav        = self._grav_comp.gravity_compensation(self._q)
        tau_pretension  = self._pretension_torque(self._q)
        if not PRETENSION_ENABLED:
            tau_pretension = np.zeros_like(tau_pretension)
        tau_raw         = tau_vmc + tau_grav - tau_pretension

        if np.any(np.abs(tau_raw) > TAU_LIMIT):
            self.get_logger().warn("Torque limit exceeded — clipping.")
        tau_cmd = np.clip(tau_raw, -TAU_LIMIT, TAU_LIMIT)

        self._publish(tau_cmd)

        msg_vf = Float64MultiArray()
        msg_vf.data = f_virtual.tolist()
        self._pub_vforce.publish(msg_vf)

        # LOGS FOR PLOTTING:
        now = time.perf_counter()
        if self._t0 is None:
            self._trim_pre_reference(now)
        self._t_log.append(now)
        self._t_wall_log.append(time.time())
        self._tau_log.append(tau_cmd.copy())
        self._tau_vmc_log.append(tau_vmc.copy())
        self._tau_grav_log.append(tau_grav.copy())
        self._tau_pre_log.append(tau_pretension.copy())
        self._tau_comp_log.append(state["tau_comp"].copy())
        self._goal_log.append(self._goal.copy())
        self._comp_pos_log.append(state["pos"].copy())
        self._comp_vel_log.append(state["vel"].copy())
        self._f_comp_log.append(state["F_comp"].copy())
        self._force_log.append(self._force_shear)
        self._force_age_log.append(
            np.nan if self._force_stamp is None else now - self._force_stamp)
        self._k_log.append([
            getattr(comp.spring, SPRING_PARAM_NAMES[i])
            for i, comp in enumerate(self._components)
        ])
        self._c_log.append([
            getattr(comp.damper, DAMPER_PARAM_NAMES[i])
            for i, comp in enumerate(self._components)
        ])
        self._q_log.append(self._q.copy())
        self._q_dot_log.append(self._q_dot.copy())

    def _install_mark_handler(self):
        """Ctrl+C in this terminal, once per moment worth marking, then once
        more to end the run.

        While a run goes this terminal is the only thing the operator can reach
        that also holds the log -- the GUI is a window and the arm is a robot --
        so Ctrl+C does both jobs. The handler runs in the main thread between
        bytecodes; the control loop is not paused by it. When the prompts are
        used up it raises KeyboardInterrupt, which is exactly what Ctrl+C did
        before, and the run is written on the way out.
        """
        self._pending_marks = list(MARK_PROMPTS)

        def on_sigint(_signum, _frame):
            if not self._pending_marks:
                raise KeyboardInterrupt          # plain Ctrl+C: end the run
            prompt = self._pending_marks.pop(0)
            label  = prompt.rstrip("? ").strip().lower() or f"mark {len(self._marks) + 1}"
            now    = time.perf_counter()
            self._marks.append((now, time.time(), label))
            when = ("before the GUI connected" if self._t0 is None
                    else f"at t = {now - self._t0:.1f} s")
            print(f"\n  [mark {len(self._marks)}] {label!r} {when}\n"
                  f"  {self._next_instruction()}", flush=True)

        signal.signal(signal.SIGINT, on_sigint)
        self.get_logger().info(
            "Ctrl+C in THIS terminal: "
            + " -> ".join([f"mark {p!r}" for p in MARK_PROMPTS]
                          + [f"end the run, write {self._base}.npz"]))

    def _next_instruction(self):
        if self._pending_marks:
            return f"Ctrl+C again to mark: {self._pending_marks[0]}"
        return f"Ctrl+C again to end the run and write {self._base}.npz (once, then wait)"

    def _trim_pre_reference(self, now):
        """Keep the pre-reference buffer to the last PRE_REFERENCE_S seconds.

        Dropped in batches: shifting a list is O(n), so paying it once per
        quarter-buffer costs far less than paying it every tick.
        """
        if not self._t_log or now - self._t_log[0] <= PRE_REFERENCE_S:
            return
        drop = max(1, len(self._t_log) // 4)
        for log in (self._t_log, self._t_wall_log, self._tau_log, self._tau_vmc_log,
                    self._tau_grav_log, self._tau_pre_log, self._tau_comp_log,
                    self._goal_log, self._comp_pos_log, self._comp_vel_log,
                    self._f_comp_log, self._force_log, self._force_age_log,
                    self._k_log, self._c_log, self._q_log, self._q_dot_log):
            del log[:drop]

    def _publish(self, tau: np.ndarray):
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_hw.publish(msg)

    # ── Plot / save ───────────────────────────────────────────────────────────

    def _out_dir(self):
        """
        Where this run is written, relative to the caller's cwd (the experiment
        directory the config came from), never this script's own directory.

        A named run (--run) goes straight to its final home under outputs/ and
        overwrites, so repeating a run replaces it. An unnamed one keeps the old
        timestamped experiments/z_<stamp> behaviour, collected afterwards with
        collect_run.py.
        """
        if self._run_name:
            return os.path.join(_CWD, "outputs", OUTPUT_FOLDER or "")
        return os.path.join(_CWD, "experiments")

    def _log_arrays(self):
        """
        Every log as one synchronised dict of arrays, or None if there is not
        enough yet.

        The control loop appends to these lists while this runs, so the lists
        can be caught at different lengths -- each is copied in one shot (a
        list copy is atomic under the GIL) and then everything is cut to the
        shortest. That is what makes a checkpoint safe to take mid-run.
        """
        logs = dict(
            t              = self._t_log,          # (n,)
            t_wall         = self._t_wall_log,     # (n,)
            tau            = self._tau_log,        # (n, 9)
            tau_vmc        = self._tau_vmc_log,    # (n, 9)
            tau_grav       = self._tau_grav_log,   # (n, 9)
            tau_pretension = self._tau_pre_log,    # (n, 9)
            tau_comp       = self._tau_comp_log,   # (n, N_VC, 9)
            goal           = self._goal_log,       # (n, 3*N_VC)
            comp_pos       = self._comp_pos_log,   # (n, N_VC, 3)
            comp_vel       = self._comp_vel_log,   # (n, N_VC, 3)
            f_comp         = self._f_comp_log,     # (n, N_VC, 3)
            force          = self._force_log,      # (n,)
            force_age      = self._force_age_log,  # (n,)
            k_params       = self._k_log,          # (n, N_VC)
            c_params       = self._c_log,          # (n, N_VC)
            q_motor        = self._q_log,          # (n, 9)
            q_motor_dot    = self._q_dot_log,      # (n, 9)
        )
        snap = {k: list(v) for k, v in logs.items()}
        n = min(len(v) for v in snap.values())
        if n < 2:
            return None
        out = {k: np.asarray(v[:n]) for k, v in snap.items()}
        # t = 0 at the first reference message, as before; the ticks before it
        # -- the arm already being driven towards DEFAULT_START -- carry
        # negative t rather than being thrown away.
        origin = self._t0 if self._t0 is not None else out["t"][0]
        out["t"]         = out["t"] - origin

        # Where the operator said something happened, on the same clock as t.
        marks = list(self._marks)
        out["marks_t"]     = np.array([m[0] - origin for m in marks], dtype=float)
        out["marks_wall"]  = np.array([m[1] for m in marks], dtype=float)
        out["marks_label"] = np.array([m[2] for m in marks], dtype=object)
        out["goal"]      = out["goal"].reshape(n, N_VC, 3)   # (n, N_VC, 3) [m]
        # tau as actually sent, vs what the model asked for before the limiter
        out["tau_raw"]   = out["tau_vmc"] + out["tau_grav"] - out["tau_pretension"]
        # Kept for readers written against the old file: the sum over elements.
        out["f_virtual"] = out["f_comp"].sum(axis=1)         # (n, 3) [N]
        return out

    def _metadata(self):
        """
        Everything needed to read the run back without this folder's config --
        including the config file itself, so a run stays interpretable after
        the numbers in it move on.
        """
        def spec(obj):
            return {"type": type(obj).__name__,
                    **{k: (v.tolist() if isinstance(v, np.ndarray) else v)
                       for k, v in getattr(obj, "__dict__", {}).items()
                       if not k.startswith("_")}}

        try:
            with open(_cfg.__file__) as fh:
                config_source = fh.read()
        except Exception:
            config_source = ""

        return dict(
            comp_names         = np.array([c.name or f"VC{i+1}"
                                           for i, c in enumerate(self._components)]),
            comp_s_params      = np.array([c.s_param for c in self._components]),
            comp_spec          = np.array([json.dumps(
                                    [{"name": c.name, "s": c.s_param,
                                      "spring": spec(c.spring), "damper": spec(c.damper)}
                                     for c in self._components])]),
            spring_param_names = np.array(SPRING_PARAM_NAMES),
            damper_param_names = np.array(DAMPER_PARAM_NAMES),
            control_freq       = np.array([CONTROL_FREQUENCY]),   # [Hz]
            tau_limit          = np.array([TAU_LIMIT]),           # [N.m]
            gravity_dir        = np.array([self._gravity_dir]),
            pretension_weights = np.asarray(PRETENSION_WEIGHTS, dtype=float),
            pretension_enabled = np.array([bool(PRETENSION_ENABLED)]),
            config_name        = np.array([_cfg.__name__]),
            config_source      = np.array([config_source]),
            run_name           = np.array([self._run_name or ""]),
            condition_targets  = np.array(list(self._cond_targets), dtype=object),
            condition_strength = np.array([np.nan if self._ramp_target is None
                                           else float(self._ramp_target)]),
            ramp_s             = np.array([self._ramp_s]),        # [s]
            # t = 0 is the first /VC_ref_pos; t < 0 is the stretch before the
            # GUI connected, when the goal was still the config's DEFAULT_START.
            t_reference_ok     = np.array([self._t0 is not None]),
        )

    def _write_npz(self, path, arrays):
        """Write through a temporary file, so an interrupt cannot leave a
        half-written .npz where a complete one used to be."""
        tmp = path + ".part.npz"
        np.savez_compressed(tmp, **arrays, **self._metadata())
        os.replace(tmp, path)

    # ── Live checkpoint ───────────────────────────────────────────────────────

    def _status_line(self, a):
        """One line of what the arm is doing right now, from the logs."""
        t    = a["t"]
        tail = t >= t[-1] - AUTOSAVE_S
        rate = int(tail.sum() / max(t[-1] - t[tail][0], 1e-9))
        clip = 100.0 * np.mean(np.any(np.abs(a["tau_raw"][tail]) > TAU_LIMIT, axis=1))
        err  = np.linalg.norm(a["comp_pos"][-1] - a["goal"][-1], axis=1)   # (N_VC,)
        f    = np.linalg.norm(a["f_comp"][-1], axis=1)                     # (N_VC,)
        per  = "  ".join(
            f"{c.name or f'VC{i+1}'}: |e|={err[i]*100:5.1f}cm |F|={f[i]:6.2f}N "
            f"{SPRING_PARAM_NAMES[i]}={a['k_params'][-1, i]:.0f}"
            for i, c in enumerate(self._components))
        age = a["force_age"][-1]
        force = ("force n/a" if np.isnan(age)
                 else f"force={a['force'][-1]:7.1f} ({age:.1f}s old)")
        return (f"[{t[-1]:7.1f}s] {len(t):7d} samples @ {rate:4d} Hz  "
                f"clip {clip:5.1f}%  {force}\n           {per}")

    def _autosave_loop(self):
        """Checkpoint the run every AUTOSAVE_S seconds, on its own thread.

        A kill -9, a power cut or a crashed driver then costs the last few
        seconds instead of the whole run. The file is separate from the final
        one and is removed once that is written, so a leftover .autosave.npz
        means the run did not exit cleanly.
        """
        while not self._stop_autosave.wait(AUTOSAVE_S):
            arrays = self._log_arrays()
            if arrays is None:
                continue
            try:
                print(self._status_line(arrays), flush=True)
            except Exception as e:
                print(f"[WARN] status line: {e!r}")
            if not SAVE_DATA or COLLECTED_DATA:
                continue
            path = os.path.join(self._out_dir(), f"{self._base}.autosave.npz")
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                self._write_npz(path, arrays)
            except Exception as e:
                print(f"[WARN] checkpoint failed ({e!r}) -- the run continues")

    # ── Final save ────────────────────────────────────────────────────────────

    def save_plot(self):
        self._stop_autosave.set()

        arrays = self._log_arrays()
        if arrays is None:
            print("[WARN] Not enough data logged to plot.")
            return

        t_arr          = arrays["t"]
        tau_arr        = arrays["tau"]
        tau_grav_arr   = arrays["tau_grav"]
        tau_raw_arr    = arrays["tau_raw"]
        goal_3d        = arrays["goal"]                       # (n, N_VC, 3)
        goal_arr       = goal_3d.reshape(len(t_arr), -1)      # (n, 3*N_VC)
        force_arr      = arrays["force"]
        f_virtual_arr  = arrays["f_virtual"]
        f_comp_arr     = arrays["f_comp"]
        k_arr          = arrays["k_params"]
        comp_pos_arrs  = [arrays["comp_pos"][:, i] for i in range(N_VC)]
        n              = len(t_arr)

        # ── Save raw data FIRST ───────────────────────────────────────────────
        # The figure is a big, slow render; the data is small and irreplaceable.
        # Writing it first means an interrupt (or a plotting failure) during the
        # save costs the plot, never the run.
        plots_dir = self._out_dir()
        os.makedirs(plots_dir, exist_ok=True)
        base      = self._base
        print(f"[..] Saving {base} to {plots_dir} -- do not interrupt")

        if SAVE_DATA:
            npz_path = os.path.join(plots_dir, f"{base}.npz")
            existed  = os.path.exists(npz_path)
            try:
                self._write_npz(npz_path, arrays)
                print(f"[OK] {'Overwrote' if existed else 'Saved'} data  -> {npz_path}"
                      f"  ({n} samples, {os.path.getsize(npz_path)/1e6:.1f} MB)")
                for j, (mt, lbl) in enumerate(zip(arrays["marks_t"], arrays["marks_label"])):
                    print(f"     mark {j + 1}: {lbl!r} at t = {mt:.1f} s")
                stale = os.path.join(plots_dir, f"{base}.autosave.npz")
                if os.path.exists(stale):
                    os.remove(stale)      # the real file is written; the checkpoint is spent
            except Exception as e:
                print(f"[FAIL] could not save data to {npz_path}: {e!r}")

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
        # With one element the components of its force are the interesting
        # thing. With several, the sum is forces applied at different points
        # added together -- meaningless as a vector -- so each element's
        # magnitude is plotted instead, and the sum only as a faint reference.
        if N_VC == 1:
            vf_colors = ["tab:blue", "tab:orange", "tab:green"]
            for k, (lbl, c) in enumerate(zip(["Fx", "Fy", "Fz"], vf_colors)):
                ax_vforce.plot(t_arr, f_virtual_arr[:, k], color=c, linewidth=1.6, label=lbl)
        else:
            f_cmap = plt.get_cmap("tab10", N_VC)
            for i, comp in enumerate(self._components):
                ax_vforce.plot(t_arr, np.linalg.norm(f_comp_arr[:, i], axis=1),
                               color=f_cmap(i), linewidth=1.6,
                               label=f"|F| {comp.name or f'VC{i+1}'}")
            ax_vforce.plot(t_arr, np.linalg.norm(f_virtual_arr, axis=1), color="gray",
                           linewidth=1.0, alpha=0.6, label="|sum|")
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
        # Shade where the limiter was active: inside those stretches the arm is
        # driven by the clip, not by the virtual model.
        clipped = np.any(np.abs(tau_raw_arr) > TAU_LIMIT, axis=1)
        if clipped.any():
            ax_tau.fill_between(t_arr, -TAU_LIMIT, TAU_LIMIT, where=clipped,
                                color="red", alpha=0.10, linewidth=0,
                                label=f"clipped ({100*clipped.mean():.0f}% of run)")
        for lim in (-TAU_LIMIT, TAU_LIMIT):
            ax_tau.axhline(lim, color="k", linewidth=0.8, alpha=0.5)
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
        png_path = os.path.join(plots_dir, f"{base}.png")
        png_existed = os.path.exists(png_path)
        try:
            fig.savefig(png_path, dpi=200, bbox_inches="tight")
            print(f"[OK] {'Overwrote' if png_existed else 'Saved'} plot  -> {png_path}")
        except Exception as e:
            print(f"[FAIL] could not save plot to {png_path}: {e!r} (the data is already written)")
        finally:
            plt.close(fig)


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
    p.add_argument("--run", metavar="NAME", default=None,
                   help="name this run: it is written straight to "
                        f"outputs/{OUTPUT_FOLDER or '<folder>'}/NAME.{{png,npz}} on exit, "
                        "overwriting a previous run of the same name. Without it, "
                        "output goes to experiments/z_<stamp> as before.")
    p.add_argument("--gravity-dir", default=GRAVITY_DIR, metavar="DIR",
                   choices=GRAVITY_DIR_CHOICES,
                   help="direction gravity pulls in the robot frame "
                        f"(default from this folder's config: {GRAVITY_DIR}). "
                        '"-z" is the arm standing straight up, "-y" the arm '
                        "rotated onto its side.")
    args, _ = p.parse_known_args(_glue_signed_values(sys.argv[1:]))  # ignore ROS args
    return args


def check_run_name(run_name):
    """Refuse a --run this config has no condition for, before anything slow runs.

    The two ButtonPressing variants no longer name the same conditions, so a
    name typed for the wrong one used to be accepted and recorded at the
    config's own strength -- usually 0 -- under a name that reads like a
    condition. Configs that define no conditions at all are unaffected: any
    name is a valid run there, as before.
    """
    conds = getattr(_cfg, "GAUSS_CONDITIONS", None)
    if not conds or not run_name or run_name in conds:
        return
    sys.exit(
        f"\n[ERROR] --run {run_name!r} is not a condition of {_cfg.__name__}.\n"
        f"        this config knows: {', '.join(f'{k} ({v:g})' for k, v in conds.items())}\n"
        f"        running it anyway would record at the config's own strength "
        f"({getattr(_cfg, 'K_GAUSS', 0):g}) under a name that reads like a condition.\n"
        f"        Drop --run for a timestamped scratch run in experiments/ instead.\n")


def main():
    args = _parse_args()
    check_run_name(args.run)      # before rclpy, before the Jacobians

    rclpy.init()
    try:
        node = VMCTorqueNode(gravity_dir=args.gravity_dir, run_name=args.run)
    except KeyboardInterrupt:
        # Startup spends ~30 s in sympy building the Jacobians. A Ctrl+C in
        # there used to dump the whole lambdify traceback; there is no run to
        # save yet, so just leave.
        print("\n[INFO] interrupted during startup -- nothing recorded")
        if rclpy.ok():
            rclpy.shutdown()
        return
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
