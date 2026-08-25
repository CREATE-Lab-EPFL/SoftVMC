"""
Boilerplate shared by the experiment configs, so each GUI_MultiVMC_Config.py
holds only the numbers that define its experiment.

Robot constants live at their source and are imported, never re-typed: the
section rest length comes from Kinematics/JacobiansMulti.py, the only place
it is defined.
"""

import sys, os

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KINE = os.path.join(_ROOT, "Kinematics")
if _KINE not in sys.path:
    sys.path.insert(0, _KINE)

from JacobiansMulti import REST_LENGTH   # [m] rest length of one section

N_SEGMENTS = 3
ARM_LENGTH = N_SEGMENTS * REST_LENGTH    # [m] straight-arm tip height


def rest_pose(s):
    """
    Straight-arm position [m] at backbone coordinate s in [0, 1].

    All three sections are identical and unbent, so height is simply linear
    in s -- s=1/3 is the first joint, s=1.0 the tip.
    """
    return np.array([0.0, 0.0, ARM_LENGTH * float(s)], dtype=float)


def default_start(components):
    """(N,3) straight-arm rest position of every virtual component."""
    return np.array([rest_pose(c.s_param) for c in components], dtype=float)


def param_info(components):
    """
    Tunable parameter of each component's spring and damper, for the GUI
    sliders and the torque node's logging:

        SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES, SPRING_PARAM_INIT, DAMPER_PARAM_INIT

    The convention is that an element's first attribute is its parameter
    (k, strength, c, ...).
    """
    def first(obj, fallback_name, fallback_val):
        d = getattr(obj, "__dict__", {})
        if d:
            return next(iter(d)), float(next(iter(d.values())))
        return fallback_name, fallback_val

    springs = [first(c.spring, "k", 1.0) for c in components]
    dampers = [first(c.damper, "c", 1.0) for c in components]
    return ([s[0] for s in springs], [d[0] for d in dampers],
            [s[1] for s in springs], [d[1] for d in dampers])
