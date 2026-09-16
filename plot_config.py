"""
Shared plotting helpers for all experiments.

Usage in every notebook:
    import sys, os
    sys.path.insert(0, '..')        # 1-level-deep notebooks
    # or
    sys.path.insert(0, '../..')     # 2-level-deep notebooks
    from plot_config import COLORS, FIG_W_SINGLE, FIG_W_DOUBLE, set_font_size

Setup block (copy into each notebook, override as needed):
    FIG_W     = 10             # inches — change freely
    FIG_H     = 6              # inches — change freely
    FONT_SIZE = 18             # pt — change to rescale all text uniformly
    LEGEND_FONT_SIZE = FONT_SIZE * 0.75  # pt — change to resize legend text only
    set_font_size(FONT_SIZE, LEGEND_FONT_SIZE)
"""

import os as _os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

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

# ── Pastel palette (same order as prop_cycle in plot_config.mplstyle) ─────────
# Softened/lightened version of the original NPG palette, same hue order.
COLORS = [
    '#8FC1D9',  # C0 pastel blue/cyan
    '#F19C90',  # C1 pastel red/coral
    '#7FC4B0',  # C2 pastel teal
    '#94A1BE',  # C3 pastel navy
    '#F4B8A8',  # C4 pastel coral
    '#AEB4D6',  # C5 pastel lavender
    '#A9DED0',  # C6 pastel light teal
    '#E28B8B',  # C7 pastel bright red/rose
    '#C2AC8E',  # C8 pastel brown/tan
    '#BFBFBF',  # C9 pastel gray
]

# Pastel diverging colormap for continuous sweeps (k-value, time, ...) — a
# softened stand-in for 'coolwarm', bookended a bit deeper than the palette
# above so the extremes stay legible against white.
PASTEL_CMAP = LinearSegmentedColormap.from_list(
    'pastel_diverging',
    ['#6F8FB0', '#AEB4D6', '#EDEAE4', '#F4B8A8', '#D97B67'],
)

# Second pastel diverging colormap (teal -> purple) for continuous sweeps that
# need to stay visually distinct from PASTEL_CMAP (blue -> red) when both kinds
# of plot are compared side by side, e.g. disturbance-rejection vs. force/k
# characterization sweeps.
PASTEL_CMAP_ALT = LinearSegmentedColormap.from_list(
    'pastel_diverging_alt',
    ['#3E8E7E', '#7FC4B0', '#EDEAE4', '#C9A8D9', '#8E5BA6'],
)


def pick_spread(values, n, bias=1.0):
    """Pick n values spread across a sorted sequence (by index, not by value),
    so a subset stays representative of the full range even when the values
    themselves aren't evenly spaced. Returns all of `values`, sorted, if
    n >= len(values).

    bias: 1.0 spreads picks evenly by index. bias < 1 concentrates more picks
    toward the end (high-value side); bias > 1 concentrates more toward the
    start. Endpoints are always included.
    """
    values = sorted(values)
    if n >= len(values):
        return values
    frac = np.linspace(0, 1, n) ** bias
    idx = sorted(set(int(round(f * (len(values) - 1))) for f in frac))
    return [values[i] for i in idx]


def set_font_size(size, legend_size=None):
    """Override all text sizes uniformly. Call after import in each notebook setup cell.

    legend_size: font size for legend text/title. Defaults to size * 0.75.
    """
    if legend_size is None:
        legend_size = size * 0.75
    plt.rcParams.update({
        'font.size':             size,
        'axes.labelsize':        size,
        'axes.titlesize':        size,
        'xtick.labelsize':       size,
        'ytick.labelsize':       size,
        'legend.fontsize':       legend_size,
        'legend.title_fontsize': legend_size,
    })
