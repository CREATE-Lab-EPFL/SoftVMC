import sys, os
# SoftVMC/ -> VMC_utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import numpy as np
import matplotlib.pyplot as plt
# import rclpy

# from SingleController import SingleController, CONTROL_FREQUENCY
from SingleVMCTaskSpace import VMC

from FK_Single import FK_Single_ee

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
CONTROL_FREQUENCY = 900          # [Hz]
STEPS             = 3000         # ~3.3 s of simulated time
dt                = 1.0 / CONTROL_FREQUENCY

# Motor dynamics (first-order model)
J_INERTIA  = 1e-5   # [kg·m²]  rotor + spool inertia
B_FRICTION = 1e-4   # [N·m·s/rad]  viscous friction

# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------
q_motor     = np.array([0.0, 0.0, 0.0])   # [q1, q2, q3] motor angles [rad]
q_motor_dot = np.array([0.0, 0.0, 0.0])   # [q1_dot, q2_dot, q3_dot] motor velocities [rad/s]

# ---------------------------------------------------------------------------
# VMC (Jacobian is built once here – SymPy symbolic diff is slow)
# ---------------------------------------------------------------------------
# convention: tau<0 -> q shorter (pulls), tau>0 -> q longer (releases)
test1 = np.array([0.01, 0.01, 0.144]) # q1, q2 equally shorter, q3 longer
test2 = np.array([0.0, 0.0, 0.145])

# sin(pi/6) = 0.5, cos(pi/6) = 0.866
q1_short = np.array([0.01, 0.0001, 0.144]) # q1 shorter (tau1 neg), q2,q3 equaly longer (tau2,tau3 pos)
q2_short = np.array([-0.01*np.sin(np.pi/6), 0.01*np.cos(np.pi/6), 0.144]) # q2 shorter (tau2 neg), q1,q3 equally longer (tau1,tau3 pos)

# q1 gets shorter instead of longer. in the trajectory, the x component is positive instead of negative 
q3_short_BUG = np.array([-0.01*np.sin(-np.pi/6), -0.01*np.cos(np.pi/6), 0.144]) # q3 shorter (tau3 neg), q1,q2 equally longer (tau1,tau2 pos)

vmc = VMC(
    spring_const=40.0,
    damp_const=1.0,
    target_pos=q2_short   # feasible position near rest

    # init pos given by motors at rest: (0, 0, 0) -> pose = (0, 0, 0.145, 0, 0, 0)
)

target_pos = vmc.target   # shorthand for plotting

# ---------------------------------------------------------------------------
# State history arrays
# ---------------------------------------------------------------------------
history_q   = np.zeros((STEPS, 3))   # motor angles   [rad]
history_qdot= np.zeros((STEPS, 3))   # motor velocities [rad/s]
history_tau = np.zeros((STEPS, 3))   # motor torques  [N·m]
history_pos = np.zeros((STEPS, 3))   # EE position x, y, z [m]
history_err = np.zeros(STEPS)        # Euclidean distance to target [m]

# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------
print("Starting simulation …")
for i in range(STEPS):
    # 1. Compute VMC torques
    tau = vmc.single_torques(q_motor, q_motor_dot)

    # 2. Motor dynamics – first-order Euler integration
    #    q_ddot = (tau - B * q_dot) / J
    q_motor_ddot = (tau - B_FRICTION * q_motor_dot) / J_INERTIA
    q_motor_dot  = q_motor_dot + q_motor_ddot * dt
    q_motor      = q_motor     + q_motor_dot  * dt

    # 3. Forward kinematics – get current EE pose
    pose = FK_Single_ee(q_motor)   # [x, y, z, yaw, pitch, roll]
    pos  = pose[:3]

    # 4. Record state
    history_q[i]    = q_motor
    history_qdot[i] = q_motor_dot
    history_tau[i]  = tau
    history_pos[i]  = pos
    history_err[i]  = np.linalg.norm(pos - target_pos)

    # 5. Progress feedback
    pos_rounded  = np.round(pose[:3], decimals=3)
    if (i<10):
        print(f"  Step {i+1}/{STEPS}  |  err = {history_err[i]:.6f} m  |  pos = {pos_rounded}")
    if (i + 1) % 300 == 0:
        print(f"  Step {i+1}/{STEPS}  |  err = {history_err[i]:.6f} m  |  pos = {pos_rounded}")

# ---------------------------------------------------------------------------
# Final state report
# ---------------------------------------------------------------------------'

final_pos = history_pos[-1]
final_err = history_err[-1]
print("\n--- Simulation complete ---")
print(f"Final EE position    : {final_pos}")
print(f"Target position      : {target_pos}")
print(f"Final position error : {final_err:.6f} m")

# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
t = np.arange(STEPS) * dt   # time axis [s]

# -- Figure 1: 3-D EE trajectory ------------------------------------------
fig1 = plt.figure(figsize=(7, 6))
ax3d = fig1.add_subplot(111, projection='3d')
ax3d.plot(history_pos[:, 0], history_pos[:, 1], history_pos[:, 2],
          lw=1.2, label='EE trajectory')
ax3d.scatter(*history_pos[0],  color='green', s=60, zorder=5, label='Start')
ax3d.scatter(*history_pos[-1], color='blue',  s=60, zorder=5, label='End')
ax3d.scatter(*target_pos,      color='red',   s=100, marker='*', zorder=5, label='Target')
ax3d.set_xlabel('x [m]')
ax3d.set_ylabel('y [m]')
ax3d.set_zlabel('z [m]')
ax3d.set_title('3-D End-Effector Trajectory')
ax3d.legend()

# -- Figure 2: EE position vs time ----------------------------------------
fig2, axes2 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
labels_pos = ['x [m]', 'y [m]', 'z [m]']
target_vals = target_pos
for k in range(3):
    axes2[k].plot(t, history_pos[:, k], label=f'pos {labels_pos[k]}')
    axes2[k].axhline(target_vals[k], color='red', ls='--', lw=0.8, label='target')
    axes2[k].set_ylabel(labels_pos[k])
    axes2[k].legend(loc='upper right', fontsize=8)
    axes2[k].grid(True)
axes2[-1].set_xlabel('Time [s]')
fig2.suptitle('End-Effector Position vs Time')
fig2.tight_layout()

# -- Figure 3: Motor torques vs time --------------------------------------
fig3, axes3 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
for k in range(3):
    axes3[k].plot(t, history_tau[:, k])
    axes3[k].set_ylabel(f'τ{k+1} [N·m]')
    axes3[k].grid(True)
axes3[-1].set_xlabel('Time [s]')
fig3.suptitle('Motor Torques vs Time')
fig3.tight_layout()

# -- Figure 4: Motor angles vs time ---------------------------------------
fig4, axes4 = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
for k in range(3):
    axes4[k].plot(t, history_q[:, k])
    axes4[k].set_ylabel(f'q{k+1} [rad]')
    axes4[k].grid(True)
axes4[-1].set_xlabel('Time [s]')
fig4.suptitle('Motor Angles vs Time')
fig4.tight_layout()

# -- Figure 5: Position error vs time (log scale) -------------------------
fig5, ax5 = plt.subplots(figsize=(9, 4))
ax5.semilogy(t, history_err)
ax5.set_xlabel('Time [s]')
ax5.set_ylabel('Position error [m]')
ax5.set_title('Position Error vs Time')
ax5.grid(True, which='both')
fig5.tight_layout()

plt.show()
