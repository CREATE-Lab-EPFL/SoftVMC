"""
read_npz.py  —  Recreate plots from a saved .npz data file.

# Save a PNG next to the .npz
python3 read_npz.py plots/k7_100_big.npz

# Open interactive window (pan/zoom)
python3 read_npz.py plots/k7_100_big.npz --show

# Crop the plot to a time window (e.g. seconds 5-40)
python3 read_npz.py plots/k7_100_big.npz --t 5 40

# Specify output PNG path explicitly
python3 read_npz.py plots/k7_100_big.npz --out /tmp/my_plot.png

# Crop to a time window and save a new .npz (e.g. seconds 5-40)
python3 read_npz.py plots/k7_100_big.npz --n 5 40
#   → writes plots/k7_100_big_t5-40.npz with only the samples in [5, 40] s
#   → all time-indexed arrays are cropped; scalars (tau_limit, etc.) are kept
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

# ── Config flags (mirrors GUI_MultiVMC_Config defaults) ───────────────────────
PLOT_Z      = True
PLOT_FORCE  = True
PLOT_VFORCE = True
PLOT_K      = True


def load(path: str):
    d = np.load(path, allow_pickle=True)
    return {k: d[k] for k in d.files}


def save_npz(d: dict, t_min: float, t_max: float, in_path: str, out_path: str = None):
    t = d["t"]
    mask = np.ones(len(t), dtype=bool)
    if t_min is not None:
        mask &= t >= t_min
    if t_max is not None:
        mask &= t <= t_max

    cropped = {}
    for k, v in d.items():
        arr = np.asarray(v)
        if arr.shape and arr.shape[0] == len(t):
            cropped[k] = arr[mask]
        else:
            cropped[k] = arr

    if out_path is None:
        base = os.path.splitext(in_path)[0]
        out_path = f"{base}_t{t_min:.0f}-{t_max:.0f}.npz"

    np.savez_compressed(out_path, **cropped)
    print(f"[OK] Saved → {out_path}  ({mask.sum()} samples)")


def plot(d: dict, t_min: float = None, t_max: float = None,
         show: bool = False, out_path: str = None):

    t        = d["t"]            # (n,)
    tau      = d["tau"]          # (n, 9)
    tau_grav = d["tau_grav"]     # (n, 9)
    goal     = d["goal"]         # (n, N_VC, 3)
    comp_pos = d["comp_pos"]     # (n, N_VC, 3)
    force    = d["force"] * 9.80665e-3  # (n,) -- raw load-cell reading is grams, convert to N

    has_vforce = "f_virtual" in d and PLOT_VFORCE
    f_virtual  = d["f_virtual"] if has_vforce else None   # (n, 3) or None

    has_k  = "k_params" in d and PLOT_K
    k_data = d["k_params"] if has_k else None             # (n, N_VC) or None

    names    = list(d.get("comp_names",   np.array([])))
    s_params = list(d.get("comp_s_params", np.array([])))
    tau_limit = float(d["tau_limit"][0]) if "tau_limit" in d else None
    ctrl_freq = float(d["control_freq"][0]) if "control_freq" in d else None

    N_VC = goal.shape[1]
    if not names:
        names = [f"VC{i+1}" for i in range(N_VC)]

    # ── Optional time crop ────────────────────────────────────────────────────
    mask = np.ones(len(t), dtype=bool)
    if t_min is not None:
        mask &= t >= t_min
    if t_max is not None:
        mask &= t <= t_max
    t        = t[mask]
    tau      = tau[mask]
    tau_grav = tau_grav[mask]
    goal     = goal[mask]
    comp_pos = comp_pos[mask]
    force    = force[mask]
    if f_virtual is not None:
        f_virtual = f_virtual[mask]
    if k_data is not None:
        k_data = k_data[mask]

    # ── Figure layout (same as save_plot) ────────────────────────────────────
    n_pos_axes = 3 if PLOT_Z else 2
    n_rows     = n_pos_axes + (1 if PLOT_FORCE else 0) + (1 if has_vforce else 0) + (1 if has_k else 0) + 1
    fig, axes  = plt.subplots(n_rows, 1, sharex=True, figsize=(13, 3.2 * n_rows))
    fig.suptitle("GUI Multi VMC: Position Components + Motor Torques", y=0.97)

    # ── Metadata box ─────────────────────────────────────────────────────────
    meta_lines = []
    for i, name in enumerate(names):
        sp = s_params[i] if i < len(s_params) else "?"
        meta_lines.append(f"{name}: s={sp:.3f}")
    extras = []
    if ctrl_freq is not None:
        extras.append(f"control_freq={ctrl_freq:.0f} Hz")
    if tau_limit is not None:
        extras.append(f"tau_limit={tau_limit} N·m")
    if extras:
        meta_lines.append("  |  ".join(extras))

    fig.subplots_adjust(top=0.82)
    fig.text(0.5, 0.92, "\n".join(meta_lines),
             ha="center", va="top", fontsize=8.5, family="monospace",
             bbox=dict(boxstyle="round", facecolor="white", alpha=0.9, edgecolor="gray"))

    # ── Axes references ───────────────────────────────────────────────────────
    ax_x      = axes[0]
    ax_y      = axes[1]
    ax_z      = axes[2] if PLOT_Z else None
    ax_force  = axes[n_pos_axes] if PLOT_FORCE else None
    ax_vforce = axes[n_pos_axes + (1 if PLOT_FORCE else 0)] if has_vforce else None
    ax_k      = axes[n_pos_axes + (1 if PLOT_FORCE else 0) + (1 if has_vforce else 0)] if has_k else None
    ax_tau    = axes[-1]

    cmap = plt.get_cmap("tab10", N_VC)

    # ── Position axes ─────────────────────────────────────────────────────────
    for i, name in enumerate(names):
        pos  = comp_pos[:, i, :]   # (n, 3)
        ref  = goal[:, i, :]       # (n, 3)
        c    = cmap(i)

        ax_x.plot(t, pos[:, 0], color=c, linewidth=2.0, label=f"{name} pos")
        ax_x.plot(t, ref[:, 0], "--", color=c, alpha=0.8, label=f"{name} goal")

        ax_y.plot(t, pos[:, 1], color=c, linewidth=2.0, label=f"{name} pos")
        ax_y.plot(t, ref[:, 1], "--", color=c, alpha=0.8, label=f"{name} goal")

        if PLOT_Z:
            ax_z.plot(t, pos[:, 2], color=c, linewidth=2.0, label=f"{name} pos")
            ax_z.plot(t, ref[:, 2], "--", color=c, alpha=0.8, label=f"{name} goal")

    for ax, label in [(ax_x, "X [m]"), (ax_y, "Y [m]")]:
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=4, fontsize=8, loc="lower right")
    if PLOT_Z:
        ax_z.set_ylabel("Z [m]")
        ax_z.grid(True, alpha=0.3)
        ax_z.legend(ncol=4, fontsize=8, loc="lower right")

    # ── Force axis ────────────────────────────────────────────────────────────
    if PLOT_FORCE:
        ax_force.plot(t, force, color="darkorange", linewidth=1.6, label="shear force")
        ax_force.set_ylabel("F_shear [N]")
        ax_force.grid(True, alpha=0.3)
        ax_force.legend(fontsize=8, loc="lower right")

    # ── Virtual forces axis ───────────────────────────────────────────────────
    if has_vforce:
        vf_colors = ["tab:blue", "tab:orange", "tab:green"]
        for k, (lbl, c) in enumerate(zip(["Fx", "Fy", "Fz"], vf_colors)):
            ax_vforce.plot(t, f_virtual[:, k], color=c, linewidth=1.6, label=lbl)
        ax_vforce.set_ylabel("F_virtual [N]")
        ax_vforce.grid(True, alpha=0.3)
        ax_vforce.legend(ncol=3, fontsize=8, loc="lower right")

    # ── Spring param (k) axis ────────────────────────────────────────────────
    if has_k:
        k_cmap = plt.get_cmap("tab10", N_VC)
        for i, name in enumerate(names):
            ax_k.plot(t, k_data[:, i], color=k_cmap(i), linewidth=1.6, label=f"{name} k")
        ax_k.set_ylabel("Spring param")
        ax_k.grid(True, alpha=0.3)
        ax_k.legend(ncol=N_VC, fontsize=8, loc="lower right")

    # ── Torque axis ───────────────────────────────────────────────────────────
    tau_cmap = plt.get_cmap("tab10", 9)
    for k in range(9):
        label = f"tau{k+11}"
        ln, = ax_tau.plot(t, tau[:, k], "-", linewidth=1.6,
                          color=tau_cmap(k), label=label)
        ax_tau.plot(t, tau_grav[:, k], "--", linewidth=1.4, alpha=0.9,
                    color=ln.get_color(), label=f"{label}_grav")

    if tau_limit is not None:
        ax_tau.axhline( tau_limit, color="red", linewidth=0.8, linestyle=":", alpha=0.6)
        ax_tau.axhline(-tau_limit, color="red", linewidth=0.8, linestyle=":", alpha=0.6)

    ax_tau.set_xlabel("Time [s]")
    ax_tau.set_ylabel("Torque [N·m]")
    ax_tau.grid(True, alpha=0.3)
    ax_tau.legend(ncol=3, fontsize=8, loc="lower right")

    # ── Output ────────────────────────────────────────────────────────────────
    if show:
        plt.tight_layout()
        plt.show()
    else:
        if out_path is None:
            # Save next to the .npz with a _replot suffix
            base = os.path.splitext(sys.argv[1])[0]
            suffix = f"_t{t_min:.0f}-{t_max:.0f}" if (t_min or t_max) else ""
            out_path = f"{base}{suffix}_replot.png"
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] Saved → {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Replot a saved VMC .npz file.")
    parser.add_argument("npz", help="Path to the .npz file")
    parser.add_argument("--show", action="store_true",
                        help="Open interactive matplotlib window instead of saving")
    parser.add_argument("--t", nargs=2, type=float, metavar=("T_MIN", "T_MAX"),
                        help="Crop time axis to [T_MIN, T_MAX] seconds")
    parser.add_argument("--out", default=None,
                        help="Output PNG path (default: <npz_stem>_replot.png)")
    parser.add_argument("--n", nargs=2, type=float, metavar=("T_MIN", "T_MAX"),
                        help="Crop to [T_MIN, T_MAX] seconds and save a new .npz")
    args = parser.parse_args()

    d = load(args.npz)

    if args.n:
        save_npz(d, t_min=args.n[0], t_max=args.n[1], in_path=args.npz)
    else:
        t_min, t_max = (args.t[0], args.t[1]) if args.t else (None, None)
        plot(d, t_min=t_min, t_max=t_max, show=args.show, out_path=args.out)


if __name__ == "__main__":
    main()
