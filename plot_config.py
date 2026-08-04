"""
Shared plotting helpers for all experiments.

Usage in every notebook:
    import sys, os
    sys.path.insert(0, '..')        # 1-level-deep notebooks
    # or
    sys.path.insert(0, '../..')     # 2-level-deep notebooks
    from plot_config import COLORS, FIG_W_SINGLE, FIG_W_DOUBLE, set_font_size

Setup block (copy into each notebook, override as needed):
    FIG_W     = 10    # inches — change freely
    FIG_H     = 6     # inches — change freely
    FONT_SIZE = 18    # pt — change to rescale all text uniformly
    set_font_size(FONT_SIZE)
"""

import os as _os
import numpy as np
import matplotlib.pyplot as plt

# Apply the shared mplstyle automatically when this module is imported.
_REPO = _os.path.dirname(_os.path.abspath(__file__))
plt.style.use(_os.path.join(_REPO, 'plot_config.mplstyle'))

# Save every figure as SVG in addition to PDF.
import matplotlib.figure as _mfig

if not getattr(_mfig.Figure, '_twin_svg_patched', False):
    _orig_savefig = _mfig.Figure.savefig

    def _savefig_with_svg(self, fname, *args, **kwargs):
        result = _orig_savefig(self, fname, *args, **kwargs)
        try:
            if isinstance(fname, (str, bytes, _os.PathLike)):
                base, ext = _os.path.splitext(_os.fspath(fname))
                if ext.lower() in ('.pdf', ''):
                    svg_kwargs = dict(kwargs)
                    svg_kwargs.pop('format', None)
                    _orig_savefig(self, base + '.svg', *args, **svg_kwargs)
        except Exception:
            pass
        return result

    _mfig.Figure.savefig = _savefig_with_svg
    _mfig.Figure._twin_svg_patched = True

# ── Nature Communications column widths (inches) ──────────────────────────────
FIG_W_SINGLE = 3.46   # 88 mm  — single column
FIG_W_DOUBLE = 7.09   # 180 mm — double column
FIG_W_1P5    = 4.72   # 120 mm — 1.5 column

# ── NPG color palette (same order as prop_cycle in plot_config.mplstyle) ──────
COLORS = [
    '#4DBBD5',  # C0 blue/cyan
    '#E64B35',  # C1 red
    '#00A087',  # C2 teal
    '#3C5488',  # C3 navy
    '#F39B7F',  # C4 coral
    '#8491B4',  # C5 lavender
    '#91D1C2',  # C6 light teal
    '#DC0000',  # C7 bright red
    '#7E6148',  # C8 brown
    '#949494',  # C9 gray
]


def set_font_size(size):
    """Override all text sizes uniformly. Call after import in each notebook setup cell."""
    plt.rcParams.update({
        'font.size':             size,
        'axes.labelsize':        size,
        'axes.titlesize':        size,
        'xtick.labelsize':       size,
        'ytick.labelsize':       size,
        'legend.fontsize':       size * 0.75,
        'legend.title_fontsize': size * 0.75,
    })
