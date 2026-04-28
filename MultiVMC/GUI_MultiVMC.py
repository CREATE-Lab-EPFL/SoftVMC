"""
GUI_MultiVMC.py
────────────
GUI node — acts as both the VC_Param publisher and VC_ref_pos publisher.
Subscribes to /joint_positions and /VMC_torque for live visualization.

Layout  (2 x 2 grid):
  ┌────────────────────┬──────────────────────────┐
  │ ① Parameter Tuning │ ③ 3-D Robot Visualisation│
  ├────────────────────┼──────────────────────────┤
  │ ② Position Control │ ④ Torque Over Time       │
  └────────────────────┴──────────────────────────┘

Run (after GUI_MultiVMC_Torque.py is already running):
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"
    python3 GUI_MultiVMC.py

Dependencies beyond ROS2 / standard library:
    pip install matplotlib plotly
"""

import os
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D          # noqa – registers 3-D projection


import plotly.graph_objects as go
from plotly.subplots import make_subplots

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from GUI_MultiVMC_Config import (
    VIRTUAL_COMPONENTS,
    DEFAULT_GOALS,
    DEFAULT_START,
    N_VC,
    GUI_COLORS,
    SPRING_PARAM_NAMES,
    DAMPER_PARAM_NAMES,
    SPRING_PARAM_INIT,
    DAMPER_PARAM_INIT,
    XY_RANGE,
    Z_RANGE,
    TAU_PLOT_WINDOW,
    FK_S_SAMPLES,
    PLOTLY_OUTPUT_DIR,
    ENABLE_PARAM_TUNING,
)

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

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, "/joint_positions", self._cb_pos,    10)
        self.create_subscription(
            Float64MultiArray, "/VMC_torque",       self._cb_torque, 10)

        # ── Shared state ──────────────────────────────────────────────────────
        self._lock            = threading.Lock()
        self.joint_positions  = np.zeros(9, dtype=float)   # radians
        self.latest_torque    = np.zeros(9, dtype=float)   # N·m
        self.torque_timestamp = None                        # perf_counter float

    # ── Callbacks (ROS spin thread) ───────────────────────────────────────────

    def _cb_pos(self, msg: Float64MultiArray):
        with self._lock:
            self.joint_positions = np.radians(np.asarray(msg.data, dtype=float))

    def _cb_torque(self, msg: Float64MultiArray):
        with self._lock:
            self.latest_torque    = np.asarray(msg.data, dtype=float)
            self.torque_timestamp = time.perf_counter()

    # ── Thread-safe getters (GUI main thread) ─────────────────────────────────

    def get_joint_positions(self) -> np.ndarray:
        with self._lock:
            return self.joint_positions.copy()

    def get_latest_torque(self):
        """Returns (torque_array, timestamp_or_None)."""
        with self._lock:
            return self.latest_torque.copy(), self.torque_timestamp

    # ── Publishers (GUI main thread) ──────────────────────────────────────────

    def publish_params(self, k_vals, c_vals):
        """Publish [k1,…,kN, c1,…,cN]."""
        msg = Float64MultiArray()
        msg.data = list(k_vals) + list(c_vals)
        self._pub_param.publish(msg)

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
        self._ref_pos   = DEFAULT_GOALS.copy()          # (N,3)  current targets
        self._k_vals    = list(SPRING_PARAM_INIT)
        self._c_vals    = list(DAMPER_PARAM_INIT)
        self._pub_active  = tk.BooleanVar(value=True)
        self._selected_vc = tk.IntVar(value=0)
        self._dragging    = False
        self._ignore_z    = False    # guard against programmatic slider set()

        # ── Logging for plotly export ──────────────────────────────────────────
        self._t0           = time.perf_counter()
        self._t_log        = deque(maxlen=self._LOG_MAXLEN)
        self._tau_log      = deque(maxlen=self._LOG_MAXLEN)
        self._robot_shape_log = deque(maxlen=self._LOG_MAXLEN)  # list of (K,3) arrays
        self._vc_attach_log   = deque(maxlen=self._LOG_MAXLEN)  # list of (N,3) arrays
        self._vc_ref_log      = deque(maxlen=self._LOG_MAXLEN)  # list of (N,3) arrays

        self._build_layout()
        self._setup_cell1()
        self._setup_cell2()
        self._setup_cell3()
        self._setup_cell4()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_loop()

    # ─────────────────────────────────────────────────────────────────────────
    # Layout
    # ─────────────────────────────────────────────────────────────────────────

    def _build_layout(self):
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=2)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=2)

        kw = dict(padding=8)
        self._cell1 = ttk.LabelFrame(self.root, text="① Parameter Tuning",        **kw)
        self._cell2 = ttk.LabelFrame(self.root, text="② Position Control",         **kw)
        self._cell3 = ttk.LabelFrame(self.root, text="③ 3-D Robot Visualisation",  **kw)
        self._cell4 = ttk.LabelFrame(self.root, text="④ Torque Over Time",          **kw)

        self._cell1.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        if not ENABLE_PARAM_TUNING:
            for child in self._cell1.winfo_children():
                child.configure(state="disabled") if hasattr(child, "configure") else None
            ttk.Label(self._cell1, text="Param tuning disabled\n(multi-param spring/damper)",
                    foreground="gray").pack(expand=True)

        self._cell2.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        self._cell3.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self._cell4.grid(row=1, column=1, sticky="nsew", padx=4, pady=4)

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

            ttk.Entry(f, textvariable=kv, width=9).grid(
                row=i+1, column=1, padx=6, pady=3)
            ttk.Entry(f, textvariable=cv, width=9).grid(
                row=i+1, column=2, padx=6, pady=3)
            ttk.Label(f, text=f"{comp.s_param:.3f}", width=7).grid(
                row=i+1, column=3, padx=6, pady=3)

            self._k_vars.append(kv)
            self._c_vars.append(cv)

        ttk.Button(f, text="Apply Parameters",
                   command=self._apply_params).grid(
            row=N_VC+1, column=0, columnspan=4, pady=8, sticky="ew")

        self._param_status = ttk.Label(f, text="")
        self._param_status.grid(row=N_VC+2, column=0, columnspan=4)

    def _apply_params(self):
        if not ENABLE_PARAM_TUNING:
            return
        try:
            k = [float(v.get()) for v in self._k_vars]
            c = [float(v.get()) for v in self._c_vars]
            self._k_vals, self._c_vals = k, c
            self.node.publish_params(k, c)
            self._param_status.config(text="✓ Published", foreground="green")
        except ValueError as exc:
            self._param_status.config(text=f"✗ {exc}", foreground="red")

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

        self._fig_xy, self._ax_xy = plt.subplots(figsize=(3.6, 3.6))
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
            sl.pack(pady=2)

            lbl = ttk.Label(z_frame, text=f"{self._ref_pos[i, 2]:.3f} m")
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
        self._dragging = True
        self._apply_xy(event)

    def _xy_motion(self, event):
        if not self._dragging or event.inaxes != self._ax_xy:
            return
        self._apply_xy(event)

    def _xy_release(self, _event):
        self._dragging = False

    def _apply_xy(self, event):
        idx = self._selected_vc.get()
        x = float(np.clip(event.xdata, *XY_RANGE))
        y = float(np.clip(event.ydata, *XY_RANGE))
        self._ref_pos[idx, 0] = x
        self._ref_pos[idx, 1] = y
        self._xy_pts[idx].set_data([x], [y])
        self._canvas_xy.draw_idle()
        if self._pub_active.get():
            self.node.publish_ref_pos(self._ref_pos)

    def _z_changed(self, idx: int, lbl: ttk.Label):
        if self._ignore_z:
            return
        z = float(self._z_vars[idx].get())
        self._ref_pos[idx, 2] = z
        lbl.config(text=f"{z:.3f} m")
        if self._pub_active.get():
            self.node.publish_ref_pos(self._ref_pos)

    def _reset_positions(self):
        self._ignore_z = True
        for i in range(N_VC):
            self._ref_pos[i] = DEFAULT_START[i].copy()
            self._xy_pts[i].set_data([DEFAULT_START[i, 0]], [DEFAULT_START[i, 1]])
            self._z_vars[i].set(float(DEFAULT_START[i, 2]))
            self._z_lbls[i].config(text=f"{DEFAULT_START[i, 2]:.3f} m")
        self._ignore_z = False
        self._canvas_xy.draw_idle()
        # Publish once after all positions are reset
        if self._pub_active.get():
            self.node.publish_ref_pos(self._ref_pos)

    # ─────────────────────────────────────────────────────────────────────────
    # Cell 3 — 3-D Robot Visualisation
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell3(self):
        self._fig3d = plt.figure(figsize=(5, 4))
        self._ax3d  = self._fig3d.add_subplot(111, projection="3d")
        ax = self._ax3d
        ax.set_xlabel("X [m]", fontsize=8)
        ax.set_ylabel("Y [m]", fontsize=8)
        ax.set_zlabel("Z [m]", fontsize=8)
        ax.set_title("Robot + VCs", fontsize=9)

        # Robot backbone
        self._robot_ln, = ax.plot([], [], [], "k-o",
                                  linewidth=2, markersize=3, label="robot")

        # Per-VC: attachment dot, reference star, dashed connector
        self._vc_attach  = []
        self._vc_ref     = []
        self._vc_conn    = []
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

        self._canvas_3d = FigureCanvasTkAgg(self._fig3d, master=self._cell3)
        self._canvas_3d.draw()
        self._canvas_3d.get_tk_widget().pack(fill="both", expand=True)

    def _update_cell3(self, q: np.ndarray, t_now: float):
        s_vals = np.linspace(0.0, 1.0, FK_S_SAMPLES)
        robot_pts = []
        for s in s_vals:
            try:
                pose = FK_Multi(q, float(s))
                robot_pts.append(pose[:3])
            except Exception:
                pass

        vc_attach_pts = np.zeros((N_VC, 3))
        vc_ref_pts    = self._ref_pos.copy()

        if robot_pts:
            rp = np.array(robot_pts)
            self._robot_ln.set_data(rp[:, 0], rp[:, 1])
            self._robot_ln.set_3d_properties(rp[:, 2])

            # Auto-scale axes
            span = max(np.ptp(rp), 0.1)
            cx, cy, cz = rp.mean(axis=0)
            for set_lim, c in zip(
                [self._ax3d.set_xlim, self._ax3d.set_ylim, self._ax3d.set_zlim],
                [cx, cy, cz],
            ):
                set_lim(c - span * 0.7, c + span * 0.7)

        for i, comp in enumerate(VIRTUAL_COMPONENTS):
            try:
                pose = FK_Multi(q, float(comp.s_param))
                attach = pose[:3]
                ref    = vc_ref_pts[i]
                vc_attach_pts[i] = attach

                self._vc_attach[i].set_data([attach[0]], [attach[1]])
                self._vc_attach[i].set_3d_properties([attach[2]])

                self._vc_ref[i].set_data([ref[0]], [ref[1]])
                self._vc_ref[i].set_3d_properties([ref[2]])

                self._vc_conn[i].set_data([attach[0], ref[0]], [attach[1], ref[1]])
                self._vc_conn[i].set_3d_properties([attach[2], ref[2]])
            except Exception:
                pass

        self._canvas_3d.draw_idle()

        # Log for plotly export
        if robot_pts:
            self._robot_shape_log.append(np.array(robot_pts))
            self._vc_attach_log.append(vc_attach_pts.copy())
            self._vc_ref_log.append(vc_ref_pts.copy())

    # ─────────────────────────────────────────────────────────────────────────
    # Cell 4 — Torque Over Time
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_cell4(self):
        # Save button
        top = ttk.Frame(self._cell4)
        top.pack(side="top", fill="x")
        ttk.Button(top, text="Save Plotly Report",
                   command=self._save_plotly).pack(side="right", padx=6, pady=2)

        self._fig_tau, self._ax_tau = plt.subplots(figsize=(5, 3))
        ax = self._ax_tau
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.set_ylabel("Torque [N·m]", fontsize=8)
        ax.set_title("Motor Torques", fontsize=9)
        ax.grid(True, alpha=0.3)

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
        tau, ts = self.node.get_latest_torque()

        self._update_cell3(q, t_now)

        if ts is not None:
            self._update_cell4(tau, t_now)

        self.root.after(self._UPDATE_MS, self._update_loop)
        self._after_id = self.root.after(self._UPDATE_MS, self._update_loop)


    # ─────────────────────────────────────────────────────────────────────────
    # Plotly export
    # ─────────────────────────────────────────────────────────────────────────

    def _save_plotly(self):
        try:
            import plotly.graph_objects as go
            from plotly.subplots import make_subplots
        except ImportError:
            messagebox.showerror("Error", "plotly not installed.\nRun: pip install plotly")
            return

        if len(self._t_log) < 2:
            messagebox.showwarning("Warning", "Not enough data logged yet.")
            return

        os.makedirs(PLOTLY_OUTPUT_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%d_%H%M")

        # ── Cell 4: Torque line chart ──────────────────────────────────────────
        self._save_plotly_torque(go, stamp)

        # ── Cell 3: 3-D animation ──────────────────────
        if len(self._robot_shape_log) > 1:
            self._save_plotly_3d(go, stamp)

        messagebox.showinfo("Saved", f"Plotly reports saved to  ./{PLOTLY_OUTPUT_DIR}/")

    def _save_plotly_torque(self, go, stamp: str):
        t_arr   = np.array(self._t_log)
        tau_arr = np.array(self._tau_log)   # (M, 9)
        cmap    = plt.get_cmap("tab10", 9)

        fig = go.Figure()
        for k in range(9):
            r, g, b, _ = cmap(k)
            color = f"rgb({int(r*255)},{int(g*255)},{int(b*255)})"
            fig.add_trace(go.Scatter(
                x=t_arr, y=tau_arr[:, k],
                mode="lines", name=f"τ{k+11}",
                line=dict(color=color, width=1.5),
            ))

        fig.update_layout(
            title="VMC Motor Torques — session log",
            xaxis_title="Time [s]",
            yaxis_title="Torque [N·m]",
            template="plotly_white",
            legend=dict(orientation="h", yanchor="bottom", y=1.02),
        )
        path = os.path.join(PLOTLY_OUTPUT_DIR, f"torque_{stamp}.html")
        fig.write_html(path, include_plotlyjs="cdn")
        print(f"[OK] Saved torque plot → {path}")

    def _save_plotly_3d(self, go, stamp: str):
        """
        3-D animation: one plotly Frame per logged timestep.
        A slider lets you scrub through time.  'Play' button animates at ~15 fps.
        """
        t_arr      = np.array(self._t_log)
        rp_list    = list(self._robot_shape_log)
        att_list   = list(self._vc_attach_log)
        ref_list   = list(self._vc_ref_log)

        n_frames = min(len(t_arr), len(rp_list))
        # Sub-sample to ≤300 frames so the HTML stays manageable
        step     = max(1, n_frames // 300)
        indices  = range(0, n_frames, step)

        def _make_traces(frame_idx):
            rp  = rp_list[frame_idx]    # (K,3)
            att = att_list[frame_idx]   # (N,3)
            ref = ref_list[frame_idx]   # (N,3)
            traces = [
                go.Scatter3d(
                    x=rp[:, 0], y=rp[:, 1], z=rp[:, 2],
                    mode="lines+markers",
                    line=dict(color="black", width=4),
                    marker=dict(size=3),
                    name="robot",
                ),
            ]
            for i in range(N_VC):
                color = GUI_COLORS[i % len(GUI_COLORS)]
                name  = VIRTUAL_COMPONENTS[i].name or f"VC{i+1}"
                # attachment
                traces.append(go.Scatter3d(
                    x=[att[i, 0]], y=[att[i, 1]], z=[att[i, 2]],
                    mode="markers", marker=dict(color=color, size=8),
                    name=f"{name} attach",
                ))
                # reference
                traces.append(go.Scatter3d(
                    x=[ref[i, 0]], y=[ref[i, 1]], z=[ref[i, 2]],
                    mode="markers", marker=dict(color=color, symbol="diamond", size=10),
                    name=f"{name} ref",
                ))
                # connector
                traces.append(go.Scatter3d(
                    x=[att[i, 0], ref[i, 0]],
                    y=[att[i, 1], ref[i, 1]],
                    z=[att[i, 2], ref[i, 2]],
                    mode="lines",
                    line=dict(color=color, width=2, dash="dot"),
                    showlegend=False,
                ))
            return traces

        # Initial frame
        init_traces = _make_traces(0)

        frames = [
            go.Frame(
                data=_make_traces(fi),
                name=f"{t_arr[fi]:.2f}",
            )
            for fi in indices
        ]

        slider_steps = [
            dict(
                args=[[f.name], dict(frame=dict(duration=0, redraw=True), mode="immediate")],
                label=f.name,
                method="animate",
            )
            for f in frames
        ]

        fig = go.Figure(data=init_traces, frames=frames)
        fig.update_layout(
            title="VMC 3-D Robot — session log",
            template="plotly_white",
            scene=dict(
                xaxis_title="X [m]",
                yaxis_title="Y [m]",
                zaxis_title="Z [m]",
                aspectmode="cube",
            ),
            updatemenus=[dict(
                type="buttons", showactive=False, y=0.05, x=0.05,
                buttons=[
                    dict(label="▶ Play",
                         method="animate",
                         args=[None, dict(frame=dict(duration=70, redraw=True),
                                          fromcurrent=True)]),
                    dict(label="⏸ Pause",
                         method="animate",
                         args=[[None], dict(frame=dict(duration=0), mode="immediate")]),
                ],
            )],
            sliders=[dict(
                active=0,
                currentvalue=dict(prefix="t = ", suffix=" s", font=dict(size=12)),
                pad=dict(t=40),
                steps=slider_steps,
            )],
        )

        path = os.path.join(PLOTLY_OUTPUT_DIR, f"robot3d_{stamp}.html")
        fig.write_html(path, include_plotlyjs="cdn")
        print(f"[OK] Saved 3-D animation → {path}")

    # ─────────────────────────────────────────────────────────────────────────
    # Close
    # ─────────────────────────────────────────────────────────────────────────

    def _on_close(self):
        self.root.after_cancel(self._after_id)
        if len(self._t_log) >= 2:
            try:
                self._save_plotly()
            except Exception as exc:
                print(f"[WARN] auto-save failed: {exc}")
        self.root.destroy()


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    ros_node = GUINode()

    # ROS2 spin in daemon thread — killed automatically when the main thread ends
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True, name="rclpy_spin"
    )
    ros_thread.start()

    root = tk.Tk()
    root.geometry("1500x920")
    root.minsize(1100, 700)

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
