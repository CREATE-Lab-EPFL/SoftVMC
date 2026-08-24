"""
Offline check of the active GUI_MultiVMC_Config.py -- no ROS, no hardware.
Prints distances, forces and the reach/repulsion torque balance for every
condition. Building the symbolic Jacobians takes ~30 s.

Everything is evaluated at the straight-arm rest pose. As the arm deflects the
body points move toward the obstacle, so the real repulsion grows above these
numbers -- treat them as sizing, not prediction.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Kinematics")))

import numpy as np

from FK_Multi import FK_Multi
from JacobiansMulti import q_motor2pose

import GUI_MultiVMC_Config as cfg


def _bisect_crossover(tau_gauss, tau_reach, lo=1e-3, hi=1e4):
    if tau_gauss(hi) < tau_reach:
        return float("nan")
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if tau_gauss(mid) < tau_reach else (lo, mid)
    return 0.5 * (lo + hi)


def report():
    q0 = np.zeros(9)
    comps = cfg.VIRTUAL_COMPONENTS
    s_tip = comps[0].s_param
    s_g   = [c.s_param for c in comps[1:]]

    print("building Jacobians ...", flush=True)
    Jf = {s: q_motor2pose(s) for s in {s_tip, *s_g}}
    pos = lambda s: np.asarray(FK_Multi(q0, s), float)[:3]
    J   = lambda s: np.array(Jf[s](*q0), float)[:3, :]

    tip = pos(s_tip)
    u   = (cfg.PRESS_TARGET - tip) / np.linalg.norm(cfg.PRESS_TARGET - tip)
    F_reach   = cfg.K_REACH * cfg.PRESS_DEPTH
    tau_reach = float(np.linalg.norm(J(s_tip).T @ (F_reach * u)))

    def gauss_force(s, k):
        d = pos(s) - cfg.OBSTACLE_POS
        return k * np.exp(-d @ d / (2 * cfg.SIGMA ** 2)) * d

    def tau_gauss(k):
        return float(np.linalg.norm(sum(J(s).T @ gauss_force(s, k) for s in s_g)))

    print(f"\nconfig            {cfg.CONFIG_NAME}")
    print(f"press target      {np.round(cfg.PRESS_TARGET, 4).tolist()}")
    print(f"obstacle          {np.round(cfg.OBSTACLE_POS, 4).tolist()}   sigma {cfg.SIGMA}")
    print(f"approach          {np.linalg.norm(cfg.PRESS_TARGET - tip):.3f} m "
          f"({np.linalg.norm(cfg.PRESS_TARGET - tip) / 0.04:.1f} s at 4 cm/s)")
    print(f"reach             k={cfg.K_REACH} N/m, depth={cfg.PRESS_DEPTH} m "
          f"-> {F_reach:.2f} N, |tau| {tau_reach:.4f} N.m")

    print("\nobstacle distances at rest")
    for s in s_g:
        d = pos(s) - cfg.OBSTACLE_POS
        print(f"  s={s:.2f}  {np.linalg.norm(d):.4f} m   decay {np.exp(-d @ d / (2 * cfg.SIGMA ** 2)):.3f}")

    print(f"\ncrossover         {_bisect_crossover(tau_gauss, tau_reach):.1f} N/m")
    print(f"\n{'condition':<10}{'strength':>10}{'|tau_g|':>10}{'ratio':>8}{'max |F|':>10}")
    for name, k in cfg.GAUSS_CONDITIONS.items():
        fmax = max(float(np.linalg.norm(gauss_force(s, k))) for s in s_g)
        print(f"{name:<10}{k:>10.1f}{tau_gauss(k):>10.4f}{tau_gauss(k) / tau_reach:>8.2f}{fmax:>9.2f} N")

    worst = max(tau_gauss(k) for k in cfg.GAUSS_CONDITIONS.values()) + tau_reach
    print(f"\nTAU_LIMIT         {cfg.TAU_LIMIT} N.m   worst nominal |tau| {worst:.4f} N.m"
          f"{'   -- OVER LIMIT' if worst > cfg.TAU_LIMIT else ''}")


if __name__ == "__main__":
    report()
