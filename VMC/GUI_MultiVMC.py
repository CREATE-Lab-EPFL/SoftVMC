"""
GUI_MultiVMC.py
────────────
GUI node — acts as both the VC_Param publisher and VC_ref_pos publisher.
Subscribes to /joint_positions and /goal_torque for live visualization.
"""

import sys, os
# Prepend the caller's cwd (not this script's own directory) so that running
# `python3 ../VMC/GUI_MultiVMC.py` from inside an experiment folder picks up
# THAT folder's GUI_MultiVMC_Config.py, not VMC/GUI_MultiVMC_Config.py.
sys.path.insert(0, os.getcwd())
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KINE = os.path.join(_ROOT, 'Kinematics')
for _p in [_ROOT, _KINE]:
    if _p not in sys.path: sys.path.insert(0, _p)

import signal
import threading
import time
from collections import deque

import numpy as np

import tkinter as tk
from tkinter import messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D          # noqa – registers 3-D projection


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray

from VMC_utils.ConfigHelpers import config_from_argv, ramp_spec, run_name_from_argv

# --config NAME picks GUI_MultiVMC_Config_NAME.py from the launch directory;
# without it the plain GUI_MultiVMC_Config.py is used, as before.
_cfg = config_from_argv()

VIRTUAL_COMPONENTS  = _cfg.VIRTUAL_COMPONENTS
DEFAULT_START       = _cfg.DEFAULT_START
N_VC                = _cfg.N_VC
SPRING_PARAM_NAMES  = _cfg.SPRING_PARAM_NAMES
DAMPER_PARAM_NAMES  = _cfg.DAMPER_PARAM_NAMES
SPRING_PARAM_INIT   = _cfg.SPRING_PARAM_INIT
DAMPER_PARAM_INIT   = _cfg.DAMPER_PARAM_INIT

# Presentation only -- a config that says nothing about how it wants to be drawn
# gets these, so headless-first configs can still be driven from the GUI.
GUI_COLORS          = getattr(_cfg, "GUI_COLORS",
                              ["red", "blue", "green", "darkorange", "purple", "cyan", "magenta"])
XY_RANGE            = getattr(_cfg, "XY_RANGE", (-0.6, 0.6))
Z_RANGE             = getattr(_cfg, "Z_RANGE", (0.0, 0.45))
TAU_PLOT_WINDOW     = getattr(_cfg, "TAU_PLOT_WINDOW", 10.0)
FK_S_SAMPLES        = getattr(_cfg, "FK_S_SAMPLES", 4)
PLOT_VIRTUAL_FORCES = getattr(_cfg, "PLOT_VIRTUAL_FORCES", True)

# ── Optional placement constraints ───────────────────────────────────────────
# All three are opt-in. A config that defines none of them takes exactly the
# same code paths as before: every goal free in x, y and z, publishing on.
#   GOAL_Z_LOCK   {name: z}          held at z; its Z slider is disabled
#   GOAL_LINK     [(name, name...)]  one shared goal -- drag one, they all move
#   LOCK_PARAMS   bool               True = k and c are the config's, read-only
#   GAUSS_TOGGLE  bool               True = offer a tickbox that switches
#                                    RAMP_COMPONENTS between their strength and 0
#   DRAG_SPEED_MAX  float            top of the drag-speed slider [cm/s]
#   END_RUN_ON_CLOSE  bool           True = closing this window also ends and
#                                    saves the torque node's run
#   HOME_BUTTON   bool               True = offer "Home & Quit" (see below)
#   PUBLISH_ON_START  bool           False = place the goals before the arm moves
_NAMES   = [c.name or f"VC{i+1}" for i, c in enumerate(VIRTUAL_COMPONENTS)]
Z_LOCKED = {_NAMES.index(k): float(v)
            for k, v in getattr(_cfg, "GOAL_Z_LOCK", {}).items() if k in _NAMES}
LINKED   = {i: [i] for i in range(N_VC)}
for _group in getattr(_cfg, "GOAL_LINK", []):
    _idx = [_NAMES.index(k) for k in _group if k in _NAMES]
    for _i in _idx:
        LINKED[_i] = list(_idx)
PUBLISH_ON_START = bool(getattr(_cfg, "PUBLISH_ON_START", True))

# An experiment whose stiffnesses are part of its definition rather than
# something to tune sets LOCK_PARAMS: the values are still shown, but they
# cannot be edited and /VC_Param is never published, so the torque node keeps
# exactly what its own config gave it -- including a strength still ramping in,
# which any /VC_Param message would cancel.
LOCK_PARAMS = bool(getattr(_cfg, "LOCK_PARAMS", False))

# --run NAME may name a level (GAUSS_CONDITIONS): its strength becomes the GUI's
# starting value, so the GUI and the torque node agree from the start.
#
# They HAVE to agree. The GUI publishes all N stiffnesses at once on "Apply
# Parameters", so a GUI that thinks the Gaussians are at the config's own
# strength -- 0, when the level is what sets them -- silently zeroes the field
# the first time anything is applied. A config whose runs are levels therefore
# demands the level here too, and says so rather than starting wrong.
_run_name = run_name_from_argv()
if getattr(_cfg, "REQUIRE_RUN_NAME", False) and not _run_name:
    _conds = getattr(_cfg, "GAUSS_CONDITIONS", {})
    raise SystemExit(
        f"\n[ERROR] --run is required by {_cfg.__name__}: the GUI has to be told which\n"
        f"        level is running, or it will show -- and apply -- the wrong stiffnesses.\n"
        f"        pick the SAME one the torque node was given: "
        f"{', '.join(f'{k} ({v:g})' for k, v in _conds.items())}\n"
        f"        e.g. python3 ../VMC/GUI_MultiVMC.py --config "
        f"_{getattr(_cfg, 'CONFIG_NAME', '')} --run {next(iter(_conds), 'NAME')}\n")

_RAMP_NAMES, _STRENGTH, _ = ramp_spec(_cfg, _run_name)
if _STRENGTH is not None:
    SPRING_PARAM_INIT = [_STRENGTH if nm in _RAMP_NAMES else v
                         for nm, v in zip(_NAMES, SPRING_PARAM_INIT)]

# One exception to LOCK_PARAMS, for showing the same reach with the field and
# without it inside one recording. The strength is walked over RAMP_S rather
# than stepped, in both directions: switching a placed field on in a single tick
# is what the start-up ramp exists to avoid, and switching it off that way drops
# the arm. Needs RAMP_COMPONENTS to have something to act on.
GAUSS_TOGGLE = bool(getattr(_cfg, "GAUSS_TOGGLE", False)) and bool(_RAMP_NAMES)
_RAMP_IDX    = [i for i, nm in enumerate(_NAMES) if nm in _RAMP_NAMES]
RAMP_S       = float(getattr(_cfg, "RAMP_S", 5.0)) or 5.0

# Ending a session by hand means putting the arm back where it started before
# anything else can run, and that cannot be done over ROS: homing drives the
# servos in extended position control over the raw serial port, which
# dynamixel_node holds for as long as it lives. So the button takes the bus off
# the driver, homes, and closes the GUI -- see _home_and_quit.
HOME_BUTTON   = bool(getattr(_cfg, "HOME_BUTTON", False))
HOME_VELOCITY = int(getattr(_cfg, "HOME_VELOCITY", 40))   # x0.229 rpm
HOME_TIMEOUT  = float(getattr(_cfg, "HOME_TIMEOUT", 120.0))

# Top of the drag-speed slider: how fast a dragged goal is allowed to chase the
# mouse. The default is what every folder had; a config that needs to cover a
# lot of ground raises it.
DRAG_SPEED_MAX = float(getattr(_cfg, "DRAG_SPEED_MAX", 10.0))

# Closing this window ends the run in the torque node's terminal too, so one
# action finishes the session instead of a Ctrl+C in each. Off by default: a
# folder where the GUI is opened and closed freely mid-run would lose its log.
END_RUN_ON_CLOSE = bool(getattr(_cfg, "END_RUN_ON_CLOSE", False))
END_RUN_TIMEOUT  = float(getattr(_cfg, "END_RUN_TIMEOUT", 30.0))   # [s] to save in

from FK_Multi import FK_Multi


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 Bridge Node
# ══════════════════════════════════════════════════════════════════════════════

class GUINode(Node):
    """
    Thin ROS2 node embedded in the GUI process.
    All data is protected by a threading.Lock so that the ROS spin thread
    and the tkinter main thread can share state safely.
    """

    def __init__(self):
        super().__init__("robot_gui_node")

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        # Transient-local: the torque node gets it even if this is published
        # microseconds before the GUI process goes away.
        self._pub_end    = self.create_publisher(Bool, "/vmc_end_run", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, "/joint_positions", self._cb_pos,    10)
        self.create_subscription(
            Float64MultiArray, "/goal_torque",      self._cb_torque, 10)
        self.create_subscription(
            Float64,           "/force_shear",      self._cb_force,   10)
        self.create_subscription(
            Float64MultiArray, "/virtual_forces",   self._cb_vforce,  10)

        # ── Shared state ──────────────────────────────────────────────────────
        self._lock            = threading.Lock()
        self.joint_positions  = np.zeros(9, dtype=float)   # radians
        self.latest_torque    = np.zeros(9, dtype=float)   # N·m
        self.torque_timestamp = None                        # perf_counter float
        self._force           = 0.0
        self._force_ts        = None
        self._vforce          = np.zeros(3, dtype=float)   # [Fx, Fy, Fz] N
        self._vforce_ts       = None

    # ── Callbacks (ROS spin thread) ───────────────────────────────────────────

    def _cb_pos(self, msg: Float64MultiArray):
        with self._lock:
            self.joint_positions = np.radians(np.asarray(msg.data, dtype=float))

    def _cb_torque(self, msg: Float64MultiArray):
        with self._lock:
            self.latest_torque    = np.asarray(msg.data, dtype=float)
            self.torque_timestamp = time.perf_counter()

    def _cb_force(self, msg: Float64):
        with self._lock:
            self._force    = float(msg.data)
            self._force_ts = time.perf_counter()

    def _cb_vforce(self, msg: Float64MultiArray):
        with self._lock:
            self._vforce    = np.asarray(msg.data, dtype=float)[:3]
            self._vforce_ts = time.perf_counter()

    # ── Thread-safe getters (GUI main thread) ─────────────────────────────────

    def get_joint_positions(self) -> np.ndarray:
        with self._lock:
            return self.joint_positions.copy()

    def get_latest_torque(self):
        """Returns (torque_array, timestamp_or_None)."""
        with self._lock:
            return self.latest_torque.copy(), self.torque_timestamp

    def get_latest_force(self):
        """Returns (force_float, timestamp_or_None)."""
        with self._lock:
            return self._force, self._force_ts

    def get_latest_virtual_forces(self):
        """Returns (vforce_array (3,), timestamp_or_None)."""
        with self._lock:
            return self._vforce.copy(), self._vforce_ts

    # ── Publishers (GUI main thread) ──────────────────────────────────────────

    def publish_params(self, k_vals, c_vals):
        """Publish [k1,…,kN, c1,…,cN]."""
        msg = Float64MultiArray()
        msg.data = list(k_vals) + list(c_vals)
        self._pub_param.publish(msg)

    def publish_end_run(self):
        """Ask the torque node to write its run and exit."""
        self._pub_end.publish(Bool(data=True))

    def publish_ref_pos(self, goals: np.ndarray):
        """Publish [x1,y1,z1,…,xN,yN,zN]. goals shape: (N,3)."""
        msg = Float64MultiArray()
        msg.data = goals.reshape(-1).tolist()
        self._pub_refpos.publish(msg)


# ══════════════════════════════════════════════════════════════════════════════
# GUI Application
# ══════════════════════════════════════════════════════════════════════════════

class RobotGUI:
    """Four-cell tkinter GUI wired to a GUINode."""

    _UPDATE_MS   = 100    # 10 Hz GUI refresh  (3-D plots are expensive)
    _LOG_MAXLEN  = 2000  # keep ≤2000 frames in memory (200 s at 10 Hz)

    def __init__(self, root: tk.Tk, node: GUINode):
        self.root = root
        self.node = node
        self.root.title("VMC Robot Controller")

        # ── Mutable state ─────────────────────────────────────────────────────
        self._ref_pos      = DEFAULT_START.copy()   # (N,3) published goal
        self._mouse_target = DEFAULT_START.copy()   # (N,3) raw mouse XY during drag
        self._k_vals    = list(SPRING_PARAM_INIT)
        self._c_vals    = list(DAMPER_PARAM_INIT)
        self._pub_active   = tk.BooleanVar(value=PUBLISH_ON_START)
        self._selected_vc  = tk.IntVar(value=0)
        self._dragging     = False
        self._run_ended    = False   # /vmc_end_run is sent once, whichever way out
        self._pending_after = []     # after() ids to cancel on the way out
        self._closing      = False   # _on_close runs once, however it is reached
        self._ignore_z     = False   # guard against programmatic slider set()
        self._goal_speed   = tk.DoubleVar(value=5.0)  # cm/s

        # Gaussian toggle: _gauss_frac is where the walk has got to, _gauss_goal
        # where the tickbox says it should end up. Equal means nothing is being
        # walked and the GUI publishes no parameters at all -- so an untouched
        # tickbox cannot cancel the torque node's own start-up ramp.
        self._gauss_on     = tk.BooleanVar(value=True)
        self._gauss_frac   = 1.0
        self._gauss_goal   = 1.0
        # The strength to come back up to. The tickbox owns WHETHER the field is
        # on, the entries own HOW STRONG it is when it is -- so this follows
        # what was last applied rather than pinning the config's value.
        self._gauss_level  = {i: SPRING_PARAM_INIT[i] for i in _RAMP_IDX}

        # ── Logging for torque / force plots ─────────────────────────────────
        self._t0          = time.perf_counter()
        self._t_log       = deque(maxlen=self._LOG_MAXLEN)
        self._tau_log     = deque(maxlen=self._LOG_MAXLEN)
        self._t_force_log = deque(maxlen=self._LOG_MAXLEN)
        self._f_log       = deque(maxlen=self._LOG_MAXLEN)
        self._ee_trail    = deque(maxlen=300)   # end-effector (x,y) trail, 30 s at 10 Hz

        self._build_layout()
        self._setup_cell1()
        self._setup_cell2()
        self._setup_cell3()
        self._setup_cell_force()
        self._setup_cell4()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_loop()

    # ─────────────────────────────────────────────────────────────────────────
    # Layout
    # ─────────────────────────────────────────────────────────────────────────

    def _build_layout(self):
        self.root.columnconfigure(0, weight=3)
        self.root.columnconfigure(1, weight=2)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=4)

        kw = dict(padding=8)
        self._cell3 = ttk.LabelFrame(self.root, text="③ 3-D Robot Visualisation", **kw)
        self._cell2 = ttk.LabelFrame(self.root, text="② Position Control",         **kw)

        self._cell3.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self._cell2.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)

        # Right column: single frame spanning both rows, 3 panels stacked inside
        self._right_frame = ttk.Frame(self.root)
        self._right_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        self._right_frame.rowconfigure(0, weight=0)   # param tuning — fixed height
        self._right_frame.rowconfigure(1, weight=1)   # force plot   — expand
        self._right_frame.rowconfigure(2, weight=2)   # torque plot  — expand (2×)
        self._right_frame.columnconfigure(0, weight=1)

        _cell4_title = "④ Virtual Forces [N]" if PLOT_VIRTUAL_FORCES else "④ Torque Over Time"
        _cell1_title = "① Parameters (fixed)" if LOCK_PARAMS else "① Parameter Tuning"
        self._cell1      = ttk.LabelFrame(self._right_frame, text=_cell1_title, **kw)
        self._cell_force = ttk.LabelFrame(self._right_frame, text="⑤ Shear Force [N]",  **kw)
        self._cell4      = ttk.LabelFrame(self._right_frame, text=_cell4_title,          **kw)

        self._cell1.grid(     row=0, column=0, sticky="nsew", pady=(0, 4))
        self._cell_force.grid(row=1, column=0, sticky="nsew", pady=(0, 4))
        self._cell4.grid(     row=2, column=0, sticky="nsew")

    # ─────────────────────────────────────────────────────────────────────────
    # Cell 1 — Parameter Tuning
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell1(self):
        f = self._cell1

        # Header
        headers = ["Component",
                   f"Spring  ({'/'.join(dict.fromkeys(SPRING_PARAM_NAMES))})",
                   f"Damper  ({'/'.join(dict.fromkeys(DAMPER_PARAM_NAMES))})",
                   "s_param"]
        for col, h in enumerate(headers):
            ttk.Label(f, text=h, font=("Helvetica", 9, "bold")).grid(
                row=0, column=col, padx=6, pady=2, sticky="w")

        self._k_vars = []
        self._c_vars = []

        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            color = GUI_COLORS[i % len(GUI_COLORS)]
            name  = comp.name or f"VC{i+1}"

            tk.Label(f, text=name, fg=color,
                     font=("Helvetica", 9, "bold"), width=12).grid(
                row=i+1, column=0, padx=6, pady=3, sticky="w")

            kv = tk.StringVar(value=f"{SPRING_PARAM_INIT[i]:.2f}")
            cv = tk.StringVar(value=f"{DAMPER_PARAM_INIT[i]:.2f}")

            entry_state = "readonly" if LOCK_PARAMS else "normal"
            ttk.Entry(f, textvariable=kv, width=9, state=entry_state).grid(
                row=i+1, column=1, padx=6, pady=3)
            ttk.Entry(f, textvariable=cv, width=9, state=entry_state).grid(
                row=i+1, column=2, padx=6, pady=3)
            ttk.Label(f, text=f"{comp.s_param:.3f}", width=7).grid(
                row=i+1, column=3, padx=6, pady=3)

            self._k_vars.append(kv)
            self._c_vars.append(cv)

        if GAUSS_TOGGLE:
            self._gauss_chk = ttk.Checkbutton(
                f, text=f"Gaussians active  (walked over {RAMP_S:.0f} s, in and out)",
                variable=self._gauss_on, command=self._gauss_toggled)
            self._gauss_chk.grid(row=N_VC+1, column=0, columnspan=4, pady=(6, 0),
                                 sticky="w", padx=6)
            # Held for RAMP_S after this window opens. The torque node may still
            # be ramping the field in, and it stops the moment any /VC_Param
            # arrives -- so a tick during that window would hand over mid-ramp
            # and step the strength from wherever the node had got to straight
            # up to the full level, on an arm with someone standing next to it.
            self._gauss_chk.state(["disabled"])
            self._hold_until_ramped(self._gauss_chk)

        if LOCK_PARAMS:
            ttk.Label(f, text="held by the config -- only the goals are placed here",
                      foreground="gray40").grid(
                row=N_VC+2, column=0, columnspan=4, pady=8)
        else:
            self._apply_btn = ttk.Button(f, text="Apply Parameters",
                                         command=self._apply_params)
            self._apply_btn.grid(row=N_VC+2, column=0, columnspan=4, pady=8,
                                 sticky="ew")
            if _STRENGTH:
                # Same reason as the tickbox: applying while the torque node is
                # still ramping the field in cancels the ramp and steps the
                # strength to the full level.
                self._apply_btn.state(["disabled"])
                self._hold_until_ramped(self._apply_btn)

        self._param_status = ttk.Label(f, text="")
        self._param_status.grid(row=N_VC+3, column=0, columnspan=4)

    def _gauss_toggled(self):
        """Tickbox moved: aim the walk at the new end, the loop does the rest."""
        self._gauss_goal = 1.0 if self._gauss_on.get() else 0.0
        levels = ", ".join(f"{self._gauss_level[i] * self._gauss_goal:g}"
                           for i in _RAMP_IDX)
        print(f"[gui] {', '.join(_RAMP_NAMES)} -> {levels} N/m "
              f"over {RAMP_S:.0f} s", flush=True)

    def _step_gauss(self):
        """One tick of the walk. Publishes only while it is actually moving."""
        if self._gauss_frac == self._gauss_goal:
            return
        step = (self._UPDATE_MS / 1000.0) / RAMP_S
        if self._gauss_goal > self._gauss_frac:
            self._gauss_frac = min(self._gauss_goal, self._gauss_frac + step)
        else:
            self._gauss_frac = max(self._gauss_goal, self._gauss_frac - step)
        for i in _RAMP_IDX:
            self._k_vals[i] = self._gauss_level[i] * self._gauss_frac
            self._k_vars[i].set(f"{self._k_vals[i]:.2f}")
        self.node.publish_params(self._k_vals, self._c_vals)

    def _hold_until_ramped(self, widget):
        """Re-enable `widget` once the torque node's start-up ramp can be over.

        The id is kept so closing the window inside that window cancels it --
        an after() left to fire into a destroyed widget is what Tk complains
        about as `invalid command name`.
        """
        self._pending_after.append(
            self.root.after(int(RAMP_S * 1000) + 500,
                            lambda: widget.state(["!disabled"])))

    def _apply_params(self):
        try:
            k = [float(v.get()) for v in self._k_vars]
            c = [float(v.get()) for v in self._c_vars]
        except ValueError as exc:
            self._param_status.config(text=f"✗ {exc}", foreground="red")
            return

        note = ""
        if GAUSS_TOGGLE:
            # The tickbox owns whether the field is on, the entries own how
            # strong it is when it is -- so what an entry means depends on
            # where the walk is:
            walking = self._gauss_frac != self._gauss_goal
            for i in _RAMP_IDX:
                if walking:
                    pass                          # mid-walk the entry shows the
                                                  # walk's own value, not a typed
                                                  # one: adopting it would set the
                                                  # level to a fraction of itself
                elif self._gauss_frac == 1.0:
                    self._gauss_level[i] = k[i]   # field on: the entry IS the level
                elif k[i] > 0.0:
                    self._gauss_level[i] = k[i]   # field off: remember for the
                                                  # next tick, do not switch it on
                k[i] = self._gauss_level[i] * self._gauss_frac
            if self._gauss_frac < 1.0:
                note = (" (Gaussians mid-walk)" if walking else
                        " (Gaussians held off -- tick the box to bring them up)")
                for i in _RAMP_IDX:
                    self._k_vars[i].set(f"{k[i]:.2f}")

        self._k_vals, self._c_vals = k, c
        self.node.publish_params(k, c)
        self._param_status.config(text="✓ Published" + note, foreground="green")

    # ─────────────────────────────────────────────────────────────────────────
    # Cell 2 — Position Control
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell2(self):
        f = self._cell2

        # ── Top controls ──────────────────────────────────────────────────────
        top = ttk.Frame(f)
        top.pack(side="top", fill="x", pady=(0, 4))

        ttk.Checkbutton(top, text="Publisher Active",
                        variable=self._pub_active).pack(side="left", padx=6)
        ttk.Button(top, text="Reset Positions",
                   command=self._reset_positions).pack(side="left", padx=6)
        if HOME_BUTTON:
            self._home_btn = ttk.Button(top, text="Home & Quit",
                                        command=self._home_and_quit)
            self._home_btn.pack(side="left", padx=6)

        # ── Drag speed cap ────────────────────────────────────────────────────
        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Label(top, text="Drag speed:").pack(side="left")
        ttk.Scale(top, from_=0.5, to=DRAG_SPEED_MAX, orient="horizontal",
                  variable=self._goal_speed, length=150).pack(side="left", padx=4)
        self._speed_lbl = ttk.Label(top, text="5.0 cm/s", width=11)
        self._speed_lbl.pack(side="left")
        self._goal_speed.trace_add(
            "write",
            lambda *_: self._speed_lbl.config(
                text=f"{self._goal_speed.get():.1f} cm/s"
            ),
        )

        # ── VC selectors ──────────────────────────────────────────────────────
        sel_row = ttk.Frame(f)
        sel_row.pack(side="top", fill="x", pady=(0, 4))
        ttk.Label(sel_row, text="Active VC:").pack(side="left", padx=4)
        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            color = GUI_COLORS[i % len(GUI_COLORS)]
            name  = comp.name or f"VC{i+1}"
            tk.Radiobutton(
                sel_row, text=name,
                variable=self._selected_vc, value=i,
                fg=color, selectcolor="lightgray",
                font=("Helvetica", 9, "bold"),
                command=self._highlight_selected,
            ).pack(side="left", padx=4)

        # ── Main area: x-y graph (left) + z sliders (right) ───────────────────
        mid = ttk.Frame(f)
        mid.pack(side="top", fill="both", expand=True)

        # ── X-Y graph ─────────────────────────────────────────────────────────
        xy_frame = ttk.Frame(mid)
        xy_frame.pack(side="left", fill="both", expand=True)

        # Figure(), not plt.subplots(): a pyplot figure carries its own Tk
        # manager, and this one is handed to a FigureCanvasTkAgg of our own --
        # two owners of the same widgets, both deleting the same Tcl commands
        # when the window closes.
        self._fig_xy = Figure(figsize=(6.0, 6.0))
        self._ax_xy  = self._fig_xy.add_subplot(111)
        self._ax_xy.set_xlim(*XY_RANGE)
        self._ax_xy.set_ylim(*XY_RANGE)
        self._ax_xy.set_xlabel("X [m]")
        self._ax_xy.set_ylabel("Y [m]")
        self._ax_xy.set_title("Click / drag to set X-Y goal", fontsize=9)
        self._ax_xy.set_aspect("equal")
        self._ax_xy.grid(True, alpha=0.3)

        self._xy_pts = []
        for i in range(N_VC):
            color = GUI_COLORS[i % len(GUI_COLORS)]
            name  = VIRTUAL_COMPONENTS[i].name or f"VC{i+1}"
            pt, = self._ax_xy.plot(
                self._ref_pos[i, 0], self._ref_pos[i, 1],
                "o", color=color, markersize=10, label=name, zorder=4,
            )
            self._xy_pts.append(pt)
        self._ax_xy.legend(fontsize=7, loc="upper right")

        self._canvas_xy = FigureCanvasTkAgg(self._fig_xy, master=xy_frame)
        self._canvas_xy.draw()
        self._canvas_xy.get_tk_widget().pack(fill="both", expand=True)

        self._hover_lbl = ttk.Label(xy_frame, text="", font=("Courier", 9))
        self._hover_lbl.pack(side="bottom", pady=2)

        self._canvas_xy.mpl_connect("button_press_event",   self._xy_press)
        self._canvas_xy.mpl_connect("motion_notify_event",  self._xy_motion)
        self._canvas_xy.mpl_connect("button_release_event", self._xy_release)

        self._highlight_selected()

        # ── Z sliders ─────────────────────────────────────────────────────────
        z_frame = ttk.LabelFrame(mid, text="Z [m]", padding=6)
        z_frame.pack(side="right", fill="y", padx=4)

        self._z_vars  = []
        self._z_lbls  = []
        for i in range(N_VC):
            color = GUI_COLORS[i % len(GUI_COLORS)]
            name  = VIRTUAL_COMPONENTS[i].name or f"VC{i+1}"

            tk.Label(z_frame, text=name, fg=color,
                     font=("Helvetica", 8, "bold")).pack()

            zv = tk.DoubleVar(value=self._ref_pos[i, 2])
            sl = ttk.Scale(
                z_frame,
                from_=Z_RANGE[1], to=Z_RANGE[0],   # top = max
                orient="vertical", variable=zv, length=130,
            )
            if i in Z_LOCKED:
                # Held by the config -- show it, do not let it be dragged.
                sl.state(["disabled"])
            sl.pack(pady=2)

            lbl = ttk.Label(z_frame,
                            text=f"{self._ref_pos[i, 2]:.3f} m"
                                 + ("  (fixed)" if i in Z_LOCKED else ""))
            lbl.pack()

            # Trace: fires on every slider move (incl. programmatic set())
            zv.trace_add(
                "write",
                lambda *_, idx=i, label=lbl: self._z_changed(idx, label),
            )
            self._z_vars.append(zv)
            self._z_lbls.append(lbl)

    # ── Cell 2 callbacks ──────────────────────────────────────────────────────

    def _highlight_selected(self):
        sel = self._selected_vc.get()
        for i, pt in enumerate(self._xy_pts):
            pt.set_markersize(14 if i == sel else 9)
            pt.set_zorder(6 if i == sel else 4)
        self._canvas_xy.draw_idle()

    def _xy_press(self, event):
        if event.inaxes != self._ax_xy:
            return
        idx = self._selected_vc.get()
        x = float(np.clip(event.xdata, *XY_RANGE))
        y = float(np.clip(event.ydata, *XY_RANGE))
        # Jump immediately on click; drag will be speed-capped from here.
        # LINKED[idx] is just [idx] unless the config shares a goal.
        for j in LINKED[idx]:
            self._ref_pos[j, 0] = x
            self._ref_pos[j, 1] = y
            self._mouse_target[j, 0] = x
            self._mouse_target[j, 1] = y
            self._xy_pts[j].set_data([x], [y])
        self._canvas_xy.draw_idle()
        if self._pub_active.get():
            self.node.publish_ref_pos(self._ref_pos)
        self._dragging = True

    def _xy_motion(self, event):
        if event.inaxes == self._ax_xy and event.xdata is not None:
            self._hover_lbl.config(text=f"x = {event.xdata:+.3f} m    y = {event.ydata:+.3f} m")
        else:
            self._hover_lbl.config(text="")
        if not self._dragging or event.inaxes != self._ax_xy:
            return
        idx = self._selected_vc.get()
        for j in LINKED[idx]:
            self._mouse_target[j, 0] = float(np.clip(event.xdata, *XY_RANGE))
            self._mouse_target[j, 1] = float(np.clip(event.ydata, *XY_RANGE))
        # Actual movement is handled in _update_loop at capped speed

    def _xy_release(self, _event):
        self._dragging = False

    def _z_changed(self, idx: int, lbl: ttk.Label):
        if self._ignore_z or idx in Z_LOCKED:
            return
        z = float(self._z_vars[idx].get())
        self._ref_pos[idx, 2] = z
        lbl.config(text=f"{z:.3f} m")
        if self._pub_active.get():
            self.node.publish_ref_pos(self._ref_pos)

    def _reset_positions(self):
        self._ignore_z = True
        for i in range(N_VC):
            self._ref_pos[i]      = DEFAULT_START[i].copy()
            self._mouse_target[i] = DEFAULT_START[i].copy()
            self._xy_pts[i].set_data([DEFAULT_START[i, 0]], [DEFAULT_START[i, 1]])
            self._z_vars[i].set(float(DEFAULT_START[i, 2]))
            self._z_lbls[i].config(text=f"{DEFAULT_START[i, 2]:.3f} m")
        self._ignore_z = False
        self._canvas_xy.draw_idle()
        # Publish once after all positions are reset
        if self._pub_active.get():
            self.node.publish_ref_pos(self._ref_pos)

    # ── End of session: home, then close ──────────────────────────────────────

    def _home_and_quit(self):
        """End the run, put the arm back on HOME_POSITION, close everything.

        This is the whole end of a session, in the order that keeps the log
        safe:

            1. the torque node is told to write its run and exit  (terminal 2)
            2. whatever holds the serial bus is stopped           (terminal 1)
            3. the arm is homed in extended position control
            4. this window closes                                 (terminal 3)

        The run is saved before anything is stopped, because saving is the one
        step that cannot be redone. Homing has to come after: it drives the
        servos over the raw serial port, which dynamixel_node holds for as long
        as it lives, so the driver is stopped either way -- and stopping it
        takes start_arm.sh with it.
        """
        if not messagebox.askokcancel(
                "Home & Quit",
                "End the session?\n\n"
                "  1. save the run and close the torque node\n"
                "  2. stop dynamixel_node (closes start_arm.sh)\n"
                f"  3. home the arm at ~{HOME_VELOCITY * 0.229:.1f} rpm\n"
                "  4. close this window"):
            return

        self._home_btn.state(["disabled"])
        self._pub_active.set(False)          # stop driving towards the goals
        self._hover_lbl.config(text="ending the run -- watch the terminals")
        threading.Thread(target=self._home_worker, daemon=True).start()

    def _home_worker(self):
        """The blocking half, off the tkinter thread."""
        # Imported here, not at module scope: the Dynamixel SDK is only needed
        # by the folders that offer this button, and a GUI used for simulation
        # should not fail to start without it.
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import MultiGoHome as gh
        ok, why = True, ""
        try:
            self._end_run("[quit] telling the torque node to save and exit")
            gh.release_port(timeout=12.0)
            gh.home_sequence(velocity=HOME_VELOCITY, timeout=HOME_TIMEOUT)
        except Exception as exc:                     # noqa: BLE001 -- reported, then shown
            ok, why = False, f"{type(exc).__name__}: {exc}"
            print(f"[home] FAILED -- {why}", flush=True)
        self.root.after(0, self._home_done, ok, why)

    def _end_run(self, why, wait=True):
        """Tell the torque node to write its run, optionally waiting for it.

        A few MB of compressed npz takes seconds, and the node exits when it is
        done -- so /goal_torque going quiet is what "saved" looks like from
        here. Home & Quit waits for it, from its worker thread, because the
        next step kills the bus and the one after moves the arm.

        Closing the window does NOT wait: nothing follows it that the save has
        to be ahead of, and this runs on the tkinter thread -- waiting there
        freezes the window for as long as it takes.
        """
        if self._run_ended:
            return
        self._run_ended = True
        print(why, flush=True)
        self.node.publish_end_run()
        if not wait:
            return
        _, ts = self.node.get_latest_torque()
        if ts is None or time.perf_counter() - ts > 5.0:
            print("[quit] no torque node was publishing -- nothing to wait for", flush=True)
            return
        deadline = time.time() + END_RUN_TIMEOUT
        while time.time() < deadline:
            _, ts = self.node.get_latest_torque()
            if ts is None or time.perf_counter() - ts > 1.5:
                print("[quit] torque node has stopped", flush=True)
                self._report_saved()
                return
            time.sleep(0.25)
        print(f"[quit] torque node still publishing after {END_RUN_TIMEOUT:.0f} s "
              "-- carrying on; check its terminal", flush=True)
        self._report_saved()

    def _report_saved(self):
        """Say whether the run is actually on disk.

        The node going quiet only means it exited -- it is not evidence that
        anything was written, and reporting it as "run saved" hid a run that
        never appeared. This looks for the file the node was told to write.
        """
        run = run_name_from_argv()
        folder = getattr(_cfg, "OUTPUT_FOLDER", "") or ""
        if not run:
            return                       # unnamed run: nothing predictable to look for
        path = os.path.join(os.getcwd(), "outputs", folder, run + ".npz")
        if os.path.exists(path):
            print(f"[quit] saved: {path}  ({os.path.getsize(path) / 1e6:.1f} MB)",
                  flush=True)
            return
        auto = path[:-4] + ".autosave.npz"
        print(f"\n[quit] *** {run}.npz WAS NOT WRITTEN ***", flush=True)
        print(f"       expected at {path}", flush=True)
        if os.path.exists(auto):
            print(f"       a checkpoint IS there: {auto}\n"
                  f"       rename it over {run}.npz to recover the run", flush=True)
        else:
            print("       and there is no checkpoint either -- read the torque "
                  "node's terminal\n       before running the level again.", flush=True)

    def _home_done(self, ok, why):
        if ok:
            print("[home] done -- closing the GUI", flush=True)
            self._on_close()
            return
        # Left where it stopped, and the driver is gone: say so rather than
        # closing over it.
        messagebox.showerror(
            "Home failed",
            f"{why}\n\nThe arm was NOT confirmed at home and dynamixel_node has "
            "been stopped. See the terminal: a delta that is a multiple of 4096 "
            "means the turn count was lost, not that a motor failed to move.")
        self._hover_lbl.config(text="")
        self._home_btn.state(["!disabled"])

    # ─────────────────────────────────────────────────────────────────────────
    # Cell 3 — 3-D Robot Visualisation  +  End-Effector X-Y / Z
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell3(self):
        # ── Split cell3 horizontally ──────────────────────────────────────────
        c3_left  = ttk.Frame(self._cell3)
        c3_right = ttk.Frame(self._cell3)
        c3_left.pack( side="left",  fill="both", expand=True)
        c3_right.pack(side="right", fill="both", expand=True)

        # ── LEFT: 3-D backbone ───────────────────────────────────────────────
        self._fig3d = Figure(figsize=(4, 3))
        self._ax3d  = self._fig3d.add_subplot(111, projection="3d")
        ax = self._ax3d
        ax.set_xlabel("X [m]", fontsize=8)
        ax.set_ylabel("Y [m]", fontsize=8)
        ax.set_zlabel("Z [m]", fontsize=8)
        ax.set_title("Robot + VCs", fontsize=9)

        self._robot_ln, = ax.plot([], [], [], "k-o",
                                  linewidth=2, markersize=3, label="robot")
        self._vc_attach = []
        self._vc_ref    = []
        self._vc_conn   = []
        for i in range(N_VC):
            color = GUI_COLORS[i % len(GUI_COLORS)]
            name  = VIRTUAL_COMPONENTS[i].name or f"VC{i+1}"
            a, = ax.plot([], [], [], "o", color=color, markersize=8,
                         label=f"{name} attach")
            r, = ax.plot([], [], [], "*", color=color, markersize=11,
                         label=f"{name} ref")
            c, = ax.plot([], [], [], "--", color=color, linewidth=1.5, alpha=0.75)
            self._vc_attach.append(a)
            self._vc_ref.append(r)
            self._vc_conn.append(c)
        ax.legend(fontsize=7, loc="upper left")

        self._canvas_3d = FigureCanvasTkAgg(self._fig3d, master=c3_left)
        self._canvas_3d.draw()
        self._canvas_3d.get_tk_widget().pack(fill="both", expand=True)

        # ── RIGHT: End-effector X-Y + Z indicator ────────────────────────────
        ee_top = ttk.Frame(c3_right)
        ee_top.pack(side="top", fill="x", pady=(0, 2))
        ttk.Label(ee_top, text="End Effector (FK s=1)",
                  font=("Helvetica", 9, "bold")).pack(side="left", padx=4)
        ttk.Button(ee_top, text="Clear trail",
                   command=self._clear_ee_trail).pack(side="right", padx=4)

        ee_mid = ttk.Frame(c3_right)
        ee_mid.pack(side="top", fill="both", expand=True)

        # X-Y scatter + trail
        ee_xy_frame = ttk.Frame(ee_mid)
        ee_xy_frame.pack(side="left", fill="both", expand=True)

        self._fig_ee = Figure(figsize=(3.5, 3.5))
        self._ax_ee  = self._fig_ee.add_subplot(111)
        aee = self._ax_ee
        aee.set_xlim(*XY_RANGE)
        aee.set_ylim(*XY_RANGE)
        aee.set_xlabel("X [m]", fontsize=8)
        aee.set_ylabel("Y [m]", fontsize=8)
        aee.set_title("EE X-Y", fontsize=9)
        aee.set_aspect("equal")
        aee.grid(True, alpha=0.3)
        self._ee_trail_ln, = aee.plot([], [], "-", color="steelblue",
                                      linewidth=1.2, alpha=0.5)
        self._ee_dot, = aee.plot([], [], "o", color="steelblue",
                                 markersize=9, zorder=5)
        self._fig_ee.tight_layout()

        self._canvas_ee = FigureCanvasTkAgg(self._fig_ee, master=ee_xy_frame)
        self._canvas_ee.draw()
        self._canvas_ee.get_tk_widget().pack(fill="both", expand=True)

        # Z read-only indicator
        ee_z_frame = ttk.LabelFrame(ee_mid, text="Z [m]", padding=4)
        ee_z_frame.pack(side="right", fill="y", padx=4, pady=4)

        self._ee_z_scale = tk.Scale(
            ee_z_frame,
            from_=Z_RANGE[1], to=Z_RANGE[0],
            orient="vertical", length=150,
            showvalue=False, state="disabled",
            troughcolor="#cce0ff",
        )
        self._ee_z_scale.pack()
        self._ee_z_lbl = ttk.Label(ee_z_frame, text="0.000 m", font=("Courier", 8))
        self._ee_z_lbl.pack()

    def _clear_ee_trail(self):
        self._ee_trail.clear()
        self._ee_trail_ln.set_data([], [])
        self._ee_dot.set_data([], [])
        self._canvas_ee.draw_idle()

    def _update_cell3(self, q: np.ndarray, t_now: float):
        # ── 3-D backbone ─────────────────────────────────────────────────────
        s_vals    = np.linspace(0.0, 1.0, FK_S_SAMPLES)
        robot_pts = []
        for s in s_vals:
            try:
                pose = FK_Multi(q, float(s))
                robot_pts.append(pose[:3])
            except Exception:
                pass

        vc_ref_pts = self._ref_pos.copy()

        if robot_pts:
            rp = np.array(robot_pts)
            self._robot_ln.set_data(rp[:, 0], rp[:, 1])
            self._robot_ln.set_3d_properties(rp[:, 2])

            span = max(np.ptp(rp), 0.1)
            cx, cy, cz = rp.mean(axis=0)
            for set_lim, c in zip(
                [self._ax3d.set_xlim, self._ax3d.set_ylim, self._ax3d.set_zlim],
                [cx, cy, cz],
            ):
                set_lim(c - span * 0.7, c + span * 0.7)

        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            try:
                pose   = FK_Multi(q, float(comp.s_param))
                attach = pose[:3]
                ref    = vc_ref_pts[i]

                self._vc_attach[i].set_data([attach[0]], [attach[1]])
                self._vc_attach[i].set_3d_properties([attach[2]])
                self._vc_ref[i].set_data([ref[0]], [ref[1]])
                self._vc_ref[i].set_3d_properties([ref[2]])
                self._vc_conn[i].set_data([attach[0], ref[0]], [attach[1], ref[1]])
                self._vc_conn[i].set_3d_properties([attach[2], ref[2]])
            except Exception:
                pass

        self._canvas_3d.draw_idle()

        # ── End-effector X-Y trail + Z indicator (s=1.0) ─────────────────────
        try:
            ee = FK_Multi(q, 1.0)
            ee_x, ee_y, ee_z = float(ee[0]), float(ee[1]), float(ee[2])

            self._ee_trail.append((ee_x, ee_y))
            trail = np.array(self._ee_trail)
            self._ee_trail_ln.set_data(trail[:, 0], trail[:, 1])
            self._ee_dot.set_data([ee_x], [ee_y])
            self._canvas_ee.draw_idle()

            self._ee_z_scale.config(state="normal")
            self._ee_z_scale.set(ee_z)
            self._ee_z_scale.config(state="disabled")
            self._ee_z_lbl.config(text=f"{ee_z:.3f} m")
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Cell Force — Shear Force Over Time
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell_force(self):
        self._fig_force = Figure(figsize=(5, 2))
        self._ax_force  = self._fig_force.add_subplot(111)
        ax = self._ax_force
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.set_ylabel("F_shear [N]", fontsize=8)
        ax.set_title("Shear Force", fontsize=9)
        ax.grid(True, alpha=0.3)
        self._force_line, = ax.plot([], [], "-", linewidth=1.4, color="darkorange",
                                    label="F_shear")
        ax.legend(fontsize=7, loc="upper left")

        self._canvas_force = FigureCanvasTkAgg(self._fig_force, master=self._cell_force)
        self._canvas_force.draw()
        self._canvas_force.get_tk_widget().pack(fill="both", expand=True)

    def _update_cell_force(self, force: float, t_now: float):
        self._t_force_log.append(t_now)
        self._f_log.append(force)

        if len(self._t_force_log) < 2:
            return

        t_arr = np.array(self._t_force_log)
        f_arr = np.array(self._f_log)

        mask = t_arr >= (t_arr[-1] - TAU_PLOT_WINDOW)
        t_w  = t_arr[mask]
        f_w  = f_arr[mask]

        self._force_line.set_data(t_w, f_w)
        self._ax_force.set_xlim(t_w[0], max(t_w[-1], t_w[0] + 1.0))
        ymax = max(float(np.max(np.abs(f_w))) * 1.1, 0.5)
        self._ax_force.set_ylim(-ymax, ymax)
        self._canvas_force.draw_idle()

    # ─────────────────────────────────────────────────────────────────────────
    # Cell 4 — Torque Over Time
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell4(self):
        self._fig_tau = Figure(figsize=(5, 3))
        self._ax_tau  = self._fig_tau.add_subplot(111)
        ax = self._ax_tau
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.grid(True, alpha=0.3)

        if PLOT_VIRTUAL_FORCES:
            ax.set_ylabel("Virtual Force [N]", fontsize=8)
            ax.set_title("Virtual Forces (task space)", fontsize=9)
            vf_colors = ["tab:blue", "tab:orange", "tab:green"]
            self._tau_lines = [
                ax.plot([], [], "-", linewidth=1.4, color=c, label=lbl)[0]
                for c, lbl in zip(vf_colors, ["Fx", "Fy", "Fz"])
            ]
            ax.legend(ncol=3, fontsize=7, loc="upper left")
        else:
            ax.set_ylabel("Torque [N·m]", fontsize=8)
            ax.set_title("Motor Torques", fontsize=9)
            cmap = plt.get_cmap("tab10", 9)
            self._tau_lines = [
                ax.plot([], [], "-", linewidth=1.2,
                        color=cmap(k), label=f"τ{k+11}")[0]
                for k in range(9)
            ]
            ax.legend(ncol=3, fontsize=7, loc="upper left")

        self._canvas_tau = FigureCanvasTkAgg(self._fig_tau, master=self._cell4)
        self._canvas_tau.draw()
        self._canvas_tau.get_tk_widget().pack(fill="both", expand=True)

    def _update_cell4(self, tau: np.ndarray, t_now: float):
        self._tau_log.append(tau.copy())
        self._t_log.append(t_now)

        if len(self._t_log) < 2:
            return

        t_arr   = np.array(self._t_log)
        tau_arr = np.array(self._tau_log)   # (M, 9)

        # Rolling window
        mask    = t_arr >= (t_arr[-1] - TAU_PLOT_WINDOW)
        t_w     = t_arr[mask]
        tau_w   = tau_arr[mask]

        for k, ln in enumerate(self._tau_lines):
            ln.set_data(t_w, tau_w[:, k])

        self._ax_tau.set_xlim(t_w[0], max(t_w[-1], t_w[0] + 1.0))
        ymax = max(float(np.max(np.abs(tau_w))) * 1.1, 0.05)
        self._ax_tau.set_ylim(-ymax, ymax)
        self._canvas_tau.draw_idle()

    # ─────────────────────────────────────────────────────────────────────────
    # Main update loop  (runs in tkinter main thread via .after())
    # ─────────────────────────────────────────────────────────────────────────

    def _update_loop(self):
        t_now  = time.perf_counter() - self._t0
        q      = self.node.get_joint_positions()
        tau, ts      = self.node.get_latest_torque()
        force, fts   = self.node.get_latest_force()

        # Speed-capped XY tracking during drag
        if self._dragging:
            idx      = self._selected_vc.get()
            max_step = (self._goal_speed.get() / 100.0) * (self._UPDATE_MS / 1000.0)
            diff     = self._mouse_target[idx, :2] - self._ref_pos[idx, :2]
            dist     = float(np.linalg.norm(diff))
            if dist > 1e-6:
                step = diff / dist * min(dist, max_step)
                for j in LINKED[idx]:
                    self._ref_pos[j, :2] += step
                    self._xy_pts[j].set_data(
                        [self._ref_pos[j, 0]], [self._ref_pos[j, 1]])
                self._canvas_xy.draw_idle()
                if self._pub_active.get():
                    self.node.publish_ref_pos(self._ref_pos)

        if GAUSS_TOGGLE:
            self._step_gauss()

        self._update_cell3(q, t_now)

        if fts is not None:
            self._update_cell_force(force, t_now)

        if PLOT_VIRTUAL_FORCES:
            vforce, vfts = self.node.get_latest_virtual_forces()
            if vfts is not None:
                self._update_cell4(vforce, t_now)
        else:
            if ts is not None:
                self._update_cell4(tau, t_now)

        self._after_id = self.root.after(self._UPDATE_MS, self._update_loop)


    # ─────────────────────────────────────────────────────────────────────────
    # Close
    # ─────────────────────────────────────────────────────────────────────────

    def _on_close(self):
        # Reached from three places -- the window manager's X, Home & Quit, and
        # the Ctrl+C handler -- and Tk will happily call it again while a
        # half-finished teardown is still unwinding.
        if self._closing:
            return
        self._closing = True
        if END_RUN_ON_CLOSE:
            self._end_run("[quit] window closing -- telling the torque node to save",
                          wait=False)
        for after_id in [self._after_id] + self._pending_after:
            try:
                self.root.after_cancel(after_id)
            except Exception:
                pass
        self._pending_after.clear()
        # Matplotlib schedules its redraws with after_idle. One still pending
        # when the widgets go away fires into nothing and Tk prints
        # `invalid command name ...idle_draw` for each -- three of them, every
        # time the GUI was closed. Cancelled here rather than left to be
        # complained about.
        for canvas in list(vars(self).values()):
            draw_id = getattr(canvas, "_idle_draw_id", None)
            if draw_id:
                try:
                    self.root.after_cancel(draw_id)
                except Exception:
                    pass
                canvas._idle_draw_id = None

        # quit() first: it only tells mainloop() to return, so main() reaches
        # its ROS shutdown even if the teardown below throws. destroy() on a
        # tree Tk is already unwinding can raise TclError, and an exception
        # escaping here left the window up and the process alive.
        try:
            self.root.quit()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception as exc:
            print(f"[gui] teardown: {exc}", flush=True)


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def _spin_safe(node):
    """Spin target that silences ExternalShutdownException on Ctrl+C."""
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    ros_node = GUINode()

    # ROS2 spin in daemon thread — killed automatically when the main thread ends
    ros_thread = threading.Thread(
        target=_spin_safe, args=(ros_node,), daemon=True, name="rclpy_spin"
    )
    ros_thread.start()

    root = tk.Tk()
    root.geometry("1600x1000")
    root.minsize(1100, 700)

    # One Ctrl+C closes the window cleanly instead of requiring two presses
    signal.signal(signal.SIGINT, lambda *_: root.quit())

    gui = RobotGUI(root, ros_node)  # noqa (keeps reference alive)

    try:
        root.mainloop()
    finally:
        try:
            ros_node.destroy_node()
        except Exception as e:
            print(f"[WARN] destroy_node: {e}")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"[WARN] rclpy.shutdown: {e}")


if __name__ == "__main__":
    main()
