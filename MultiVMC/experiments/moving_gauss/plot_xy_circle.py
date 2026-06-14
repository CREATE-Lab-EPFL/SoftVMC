"""
plot_xy_circle.py
─────────────────
Side-by-side x-y trajectory comparison during the circling phase.
Left panel: k_lin = 5 N/m   (k5_moving_gauss.npz)
Right panel: k_lin = 50 N/m  (k50_moving_gauss.npz)

Each panel shows:
  • Robot end-effector path, time-colored blue→red
  • Gaussian goal path, time-colored (same cmap, lower alpha)
  • Dashed circle at R=10 cm for reference
  • Origin marker

Run from this directory:
    python3 plot_xy_circle.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
import os

# ── Helpers ───────────────────────────────────────────────────────────────────

def load_circle_window(path: str):
    """Return (t, ee_xy, gauss_xy) arrays trimmed to the circling phase."""
    d = np.load(path)
    t        = d['t']
    goal     = d['goal']      # (n, N_VC, 3)
    comp_pos = d['comp_pos']  # (n, N_VC, 3)

    # Circle starts when |gauss_goal_y| first exceeds 2 mm, ends when it last does
    gauss_goal_y = goal[:, 1, 1]
    mask = np.abs(gauss_goal_y) > 0.002
    idx_start = np.where(mask)[0][0]
    idx_end   = np.where(mask)[0][-1]

    sl = slice(idx_start, idx_end + 1)
    t_w        = t[sl] - t[idx_start]          # zero-based time
    ee_xy      = comp_pos[sl, 0, :2]           # (n, 2)  lin_seg3 end-effector
    gauss_xy   = goal[sl, 1, :2]               # (n, 2)  gaussian goal

    return t_w, ee_xy, gauss_xy


def make_segments(xy: np.ndarray):
    """Convert (n, 2) path into (n-1, 2, 2) line segments for LineCollection."""
    pts = xy.reshape(-1, 1, 2)
    return np.concatenate([pts[:-1], pts[1:]], axis=1)


def draw_panel(ax, t, ee_xy, gauss_xy, title: str, cmap_name='coolwarm'):
    cmap = plt.colormaps[cmap_name]
    norm = mcolors.Normalize(vmin=0, vmax=t[-1])

    # Reference circle
    theta = np.linspace(0, 2 * np.pi, 300)
    ax.plot(0.10 * np.cos(theta), 0.10 * np.sin(theta),
            '--', color='gray', linewidth=0.8, alpha=0.5, label='R=10 cm ref')

    # Origin
    ax.plot(0, 0, 'k+', markersize=8, markeredgewidth=1.5, label='Origin')

    # Gaussian goal path
    segs_g = make_segments(gauss_xy)
    lc_g = LineCollection(segs_g, cmap=cmap, norm=norm,
                          linewidth=1.0, alpha=0.45, zorder=2)
    lc_g.set_array(0.5 * (t[:-1] + t[1:]))
    ax.add_collection(lc_g)
    # ghost handle for legend
    ax.plot([], [], color=cmap(0.5), linewidth=1.0, alpha=0.55, label='Gauss goal')

    # End-effector path
    segs_e = make_segments(ee_xy)
    lc_e = LineCollection(segs_e, cmap=cmap, norm=norm,
                          linewidth=1.8, zorder=3)
    lc_e.set_array(0.5 * (t[:-1] + t[1:]))
    ax.add_collection(lc_e)
    ax.plot([], [], color=cmap(0.0), linewidth=1.8, label='EE start')
    ax.plot([], [], color=cmap(1.0), linewidth=1.8, label='EE end')

    ax.set_title(title, fontsize=11)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    ax.legend(fontsize=7.5, loc='upper right', framealpha=0.9)
    ax.grid(True, linewidth=0.4, alpha=0.5)

    lim = 0.16
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)

    return lc_e  # return for shared colorbar


# ── Main ──────────────────────────────────────────────────────────────────────

HERE = os.path.dirname(os.path.abspath(__file__))

files = {
    'k_lin = 5 N/m':  os.path.join(HERE, 'k5_moving_gauss.npz'),
    'k_lin = 50 N/m': os.path.join(HERE, 'k50_moving_gauss.npz'),
}

fig, axes = plt.subplots(1, 2, figsize=(10, 5))
fig.suptitle('End-effector trajectory during Gaussian circling  (blue→red = time)',
             fontsize=12)

last_lc = None
last_t  = None

for ax, (title, path) in zip(axes, files.items()):
    t, ee_xy, gauss_xy = load_circle_window(path)
    lc = draw_panel(ax, t, ee_xy, gauss_xy, title)
    last_lc = lc
    last_t  = t

# Shared colorbar (time axis)
cbar = fig.colorbar(last_lc, ax=axes, orientation='vertical',
                    fraction=0.02, pad=0.04, label='Time [s]')

out = os.path.join(HERE, 'xy_circle_comparison.png')
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f"Saved → {out}")
plt.show()
