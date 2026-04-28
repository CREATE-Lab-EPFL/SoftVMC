import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import sys, os

# Add root folder to Python path to find Bimanual_Parameters
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
try:
    from Bimanual_Parameters import R_WB_L, R_WB_R, t_WB_L, t_WB_R
    print("✅ Kinematic Parameters loaded successfully!")
except ImportError:
    R_WB_R = np.eye(3); t_WB_R = np.array([[0.15, 0, 0]]).T
    R_WB_L = np.eye(3); t_WB_L = np.array([[-0.15, 0, 0]]).T
    print("⚠️ Warning: Bimanual_Parameters not found, using default values.")

class GuiPublisher(Node):
    def __init__(self):
        super().__init__('gui_goal_publisher')
        
        self.publisher_ = self.create_publisher(Float64MultiArray, '/goal_simulation', 10)
        self.gain_pub = self.create_publisher(Float64MultiArray, '/vmc_gains', 10)
        
        self.sub_R = self.create_subscription(Float64MultiArray, '/debug_vmc_R', self.callback_R, 10)
        self.sub_L = self.create_subscription(Float64MultiArray, '/debug_vmc_L', self.callback_L, 10)
        
        self.latest_actual_R = None
        self.latest_actual_L = None

    def callback_R(self, msg):
        self.latest_actual_R = tuple(msg.data)

    def callback_L(self, msg):
        self.latest_actual_L = tuple(msg.data)

    def send_goal(self, pts_R, pts_L):
        msg = Float64MultiArray()
        data = []
        for p in pts_R: data.extend([float(p['x'].get()), float(p['y'].get()), float(p['z'].get()), 0.0, 0.0, 0.0])
        for p in pts_L: data.extend([float(p['x'].get()), float(p['y'].get()), float(p['z'].get()), 0.0, 0.0, 0.0])
        msg.data = data
        self.publisher_.publish(msg)

    def send_gains(self, pts_R, pts_L):
        msg = Float64MultiArray()
        data = []
        for p in pts_R: data.extend([float(p['k'].get()), float(p['d'].get())])
        for p in pts_L: data.extend([float(p['k'].get()), float(p['d'].get())])
        msg.data = data
        self.gain_pub.publish(msg)

class GenericRobotGUI:
    def __init__(self, root, num_points=4):
        self.root = root
        self.num_points = num_points
        self.root.title("🦾 Bimanual Ground Control Station")
        self.root.geometry("1450x850")
        
        self.ros_node = GuiPublisher()
        self.main_container = ttk.Frame(root)
        self.main_container.pack(fill=tk.BOTH, expand=True)
        
        self.live_mode_var = tk.BooleanVar(value=True) 
        self.z_total = 0.580
        self.setup_ui()
        
        # Start State-Based Control Loops
        self.update_ros_loop()
        self.update_plot_loop()

    def get_smart_s_params(self, num_pts):
        if num_pts == 3: return [0.5, 0.75, 1.0]
        elif num_pts == 4: return [0.5, 0.75, 0.9, 1.0]
        else: return [ (i+1)/num_pts for i in range(num_pts) ]

    def setup_ui(self):
        self.left_panel = ttk.Frame(self.main_container)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=5)
        
        header = ttk.Frame(self.left_panel)
        header.pack(fill='x', pady=5)
        ttk.Label(header, text="Points per Arm:").pack(side='left')
        self.point_count_var = tk.IntVar(value=self.num_points)
        ttk.Spinbox(header, from_=1, to=10, textvariable=self.point_count_var, width=5, command=self.rebuild_points).pack(side='left', padx=10)
        ttk.Button(header, text="RESET HOME", command=self.reset_home).pack(side='right')

        mode_frame = ttk.LabelFrame(self.left_panel, text="Transmission Mode", padding=10)
        mode_frame.pack(fill='x', pady=5)
        ttk.Radiobutton(mode_frame, text="Live", variable=self.live_mode_var, value=True).pack(side='left', expand=True)
        ttk.Radiobutton(mode_frame, text="Preview", variable=self.live_mode_var, value=False).pack(side='left', expand=True)
        self.btn_send = tk.Button(mode_frame, text="🚀 SEND COMMAND TO ROBOT", bg="#4CAF50", fg="white", font=("Helvetica", 10, "bold"), command=self.send_to_robot)
        self.btn_send.pack(fill='x', pady=5, ipady=5)

        quick_frame = ttk.LabelFrame(self.left_panel, text="Quick Actions (One-Shot)", padding=5)
        quick_frame.pack(fill='x', pady=5)
        ttk.Button(quick_frame, text="👯 CLONE", command=self.clone_arms).pack(side='left', expand=True, padx=2)
        ttk.Button(quick_frame, text="🦋 MIRROR", command=self.mirror_arms).pack(side='left', expand=True, padx=2)

        # --- LIVE SYNCHRONIZATION FRAME ---
        sync_frame = ttk.LabelFrame(self.left_panel, text="Live Link (Master: Right)", padding=5)
        sync_frame.pack(fill='x', pady=5)
        self.sync_mode_var = tk.StringVar(value="none")
        ttk.Radiobutton(sync_frame, text="Independent", variable=self.sync_mode_var, value="none").pack(side='left', expand=True)
        ttk.Radiobutton(sync_frame, text="Live Clone", variable=self.sync_mode_var, value="clone").pack(side='left', expand=True)
        ttk.Radiobutton(sync_frame, text="Live Mirror", variable=self.sync_mode_var, value="mirror").pack(side='left', expand=True)

        canvas = tk.Canvas(self.left_panel, width=580); canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll = ttk.Scrollbar(self.left_panel, orient="vertical", command=canvas.yview); scroll.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.configure(yscrollcommand=scroll.set)
        self.slider_frame = ttk.Frame(canvas); canvas.create_window((0,0), window=self.slider_frame, anchor="nw")

        self.col_L = ttk.LabelFrame(self.slider_frame, text="LEFT ARM (Green / White)")
        self.col_L.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        self.col_R = ttk.LabelFrame(self.slider_frame, text="RIGHT ARM (Red / Black)")
        self.col_R.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        self.pts_vars_R, self.pts_vars_L = [], []
        s_list = self.get_smart_s_params(self.num_points)
        
        default_k = [100.0, 100.0, 35.0, 45.0]
        default_d = [4.0, 3.0, 2.0, 2.0]
        
        for i in range(self.num_points):
            z_val = self.z_total * s_list[i]
            k_init = default_k[i] if i < len(default_k) else 50.0
            d_init = default_d[i] if i < len(default_d) else 2.0
            
            v_R = {'x': tk.DoubleVar(value=0.0), 'y': tk.DoubleVar(value=0.0), 'z': tk.DoubleVar(value=z_val),
                   'k': tk.DoubleVar(value=k_init), 'd': tk.DoubleVar(value=d_init),
                   'k_str': tk.StringVar(value=f"{k_init:.0f}"), 'd_str': tk.StringVar(value=f"{d_init:.1f}")}
                   
            v_L = {'x': tk.DoubleVar(value=0.0), 'y': tk.DoubleVar(value=0.0), 'z': tk.DoubleVar(value=z_val),
                   'k': tk.DoubleVar(value=k_init), 'd': tk.DoubleVar(value=d_init),
                   'k_str': tk.StringVar(value=f"{k_init:.0f}"), 'd_str': tk.StringVar(value=f"{d_init:.1f}")}
            
            self.pts_vars_R.append(v_R); self.pts_vars_L.append(v_L)
            
            s_label = f"Point {i+1} (s={s_list[i]})"
            
            self.add_point_ui(self.col_L, s_label, v_L)
            self.add_point_ui(self.col_R, s_label, v_R)

        # Bind tracing logic to enable Master-Slave Live Links
        self.setup_traces()

        self.slider_frame.update_idletasks(); canvas.config(scrollregion=canvas.bbox("all"))

        self.fig = plt.Figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas_tk = FigureCanvasTkAgg(self.fig, master=self.main_container)
        self.canvas_tk.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def add_point_ui(self, parent, label, vars):
        group = ttk.LabelFrame(parent, text=label); group.pack(fill='x', padx=5, pady=2)
        
        for a in ['x', 'y', 'z']:
            f = ttk.Frame(group); f.pack(fill='x')
            ttk.Label(f, text=a.upper(), width=2).pack(side='left')
            ttk.Scale(f, from_=-0.2, to=0.6, variable=vars[a]).pack(side='left', fill='x', expand=True)
            
        f_k = ttk.Frame(group); f_k.pack(fill='x')
        ttk.Label(f_k, text="K", width=2, foreground="blue").pack(side='left')
        ttk.Scale(f_k, from_=0.0, to=200.0, variable=vars['k'], 
                  command=lambda v, var=vars['k_str']: var.set(f"{float(v):.0f}")).pack(side='left', fill='x', expand=True)
        ttk.Label(f_k, textvariable=vars['k_str'], width=4, font=("Helvetica", 8, "bold")).pack(side='left')
        
        f_d = ttk.Frame(group); f_d.pack(fill='x')
        ttk.Label(f_d, text="D", width=2, foreground="red").pack(side='left')
        ttk.Scale(f_d, from_=0.0, to=10.0, variable=vars['d'], 
                  command=lambda v, var=vars['d_str']: var.set(f"{float(v):.1f}")).pack(side='left', fill='x', expand=True)
        ttk.Label(f_d, textvariable=vars['d_str'], width=4, font=("Helvetica", 8, "bold")).pack(side='left')

    # --- TRACING LOGIC FOR LIVE MASTER-SLAVE LINKING ---
    def setup_traces(self):
        for i in range(self.num_points):
            v_R = self.pts_vars_R[i]
            v_L = self.pts_vars_L[i]
            for key in ['x', 'y', 'z', 'k', 'd']:
                self._bind_trace(v_R, v_L, key)

    def _bind_trace(self, v_R, v_L, key):
        def on_change(*args):
            mode = self.sync_mode_var.get()
            if mode == "none": return
            
            val = v_R[key].get()
            
            if mode == "clone":
                v_L[key].set(val)
            elif mode == "mirror":
                if key in ['x', 'y']:
                    v_L[key].set(-val)
                else:
                    v_L[key].set(val)
            
            if key == 'k':
                v_L['k_str'].set(f"{v_L['k'].get():.0f}")
            elif key == 'd':
                v_L['d_str'].set(f"{v_L['d'].get():.1f}")
                
        v_R[key].trace_add('write', on_change)
    # --------------------------------------------------

    def send_to_robot(self): 
        self.ros_node.send_goal(self.pts_vars_R, self.pts_vars_L)
        self.ros_node.send_gains(self.pts_vars_R, self.pts_vars_L)

    def clone_arms(self):
        for vR, vL in zip(self.pts_vars_R, self.pts_vars_L):
            for k in ['x', 'y', 'z', 'k', 'd']:
                vL[k].set(vR[k].get())
            vL['k_str'].set(vR['k_str'].get())
            vL['d_str'].set(vR['d_str'].get())
        if not self.live_mode_var.get():
            self.send_to_robot()

    def mirror_arms(self):
        for vR, vL in zip(self.pts_vars_R, self.pts_vars_L):
            vL['x'].set(-vR['x'].get())
            vL['y'].set(-vR['y'].get())
            vL['z'].set(vR['z'].get())
            vL['k'].set(vR['k'].get())
            vL['d'].set(vR['d'].get())
            vL['k_str'].set(vR['k_str'].get())
            vL['d_str'].set(vR['d_str'].get())
        if not self.live_mode_var.get():
            self.send_to_robot()

    def get_global_points(self, pts_vars, R, t):
        xs, ys, zs = [float(t[0])], [float(t[1])], [float(t[2])]
        for v in pts_vars:
            p_local = np.array([[v['x'].get(), v['y'].get(), v['z'].get()]]).T
            p_global = (R @ p_local) + t
            xs.append(float(p_global[0])); ys.append(float(p_global[1])); zs.append(float(p_global[2]))
        return xs, ys, zs

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim([-0.4, 0.4]); self.ax.set_ylim([-0.4, 0.4]); self.ax.set_zlim([0, 0.7])
        self.ax.set_title("Bimanual Global Workspace")

        xR_t, yR_t, zR_t = self.get_global_points(self.pts_vars_R, R_WB_R, t_WB_R)
        xL_t, yL_t, zL_t = self.get_global_points(self.pts_vars_L, R_WB_L, t_WB_L)
        
        self.ax.plot(xR_t, yR_t, zR_t, 'r--', alpha=0.5, label='Target R')
        self.ax.plot(xL_t, yL_t, zL_t, 'g--', alpha=0.5, label='Target L')

        data_R = self.ros_node.latest_actual_R
        data_L = self.ros_node.latest_actual_L

        if data_R is not None and len(data_R) >= 12:
            xs, ys, zs = [float(t_WB_R[0])], [float(t_WB_R[1])], [float(t_WB_R[2])]
            for i in range(len(data_R) // 12):
                xs.append(data_R[i*12]); ys.append(data_R[i*12+1]); zs.append(data_R[i*12+2])
            
            self.ax.plot(xs, ys, zs, color='black', alpha=0.45, linewidth=25, solid_capstyle='round', label='Volume R')
            self.ax.plot(xs, ys, zs, 'r-', linewidth=3, marker='o', label='Actual R')
        else:
            self.ax.plot(xR_t, yR_t, zR_t, color='black', alpha=0.45, linewidth=25, solid_capstyle='round')

        if data_L is not None and len(data_L) >= 12:
            xs, ys, zs = [float(t_WB_L[0])], [float(t_WB_L[1])], [float(t_WB_L[2])]
            for i in range(len(data_L) // 12):
                xs.append(data_L[i*12]); ys.append(data_L[i*12+1]); zs.append(data_L[i*12+2])
            
            self.ax.plot(xs, ys, zs, color='lightgray', alpha=0.7, linewidth=25, solid_capstyle='round', label='Volume L')
            self.ax.plot(xs, ys, zs, 'g-', linewidth=3, marker='o', label='Actual L')
        else:
            self.ax.plot(xL_t, yL_t, zL_t, color='lightgray', alpha=0.7, linewidth=25, solid_capstyle='round')

        leg = self.ax.legend(loc='upper left', fontsize='small')
        for line in leg.get_lines():
            if line.get_linewidth() > 5:
                line.set_linewidth(6)

        self.canvas_tk.draw_idle()

    def rebuild_points(self):
        self.num_points = self.point_count_var.get()
        for w in self.main_container.winfo_children(): w.destroy()
        self.setup_ui()

    def reset_home(self):
        s_list = self.get_smart_s_params(self.num_points)
        default_k = [100.0, 100.0, 35.0, 45.0]
        default_d = [4.0, 3.0, 2.0, 2.0]
        
        for i in range(self.num_points):
            z_val = self.z_total * s_list[i]
            k_init = default_k[i] if i < len(default_k) else 50.0
            d_init = default_d[i] if i < len(default_d) else 2.0
            
            for v in [self.pts_vars_R[i], self.pts_vars_L[i]]:
                v['x'].set(0.0); v['y'].set(0.0); v['z'].set(z_val)
                v['k'].set(k_init); v['d'].set(d_init)
                v['k_str'].set(f"{k_init:.0f}")
                v['d_str'].set(f"{d_init:.1f}")
                
        if not self.live_mode_var.get():
            self.send_to_robot()

    def update_ros_loop(self):
        for _ in range(3):
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
            
        if self.live_mode_var.get():
            self.send_to_robot()
            
        self.root.after(20, self.update_ros_loop)

    def update_plot_loop(self):
        self.update_plot()
        self.root.after(66, self.update_plot_loop)

def main():
    rclpy.init(); root = tk.Tk(); app = GenericRobotGUI(root); root.mainloop()

if __name__ == '__main__': main()