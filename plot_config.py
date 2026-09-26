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

# ── Pastel SEQUENTIAL colormaps (monotonic lightness, no washed-out midpoint) ──
# PASTEL_CMAP/PASTEL_CMAP_ALT above are diverging (light midpoint) shaped, which
# is the wrong tool for a swept scalar like a spring stiffness k: the values
# near the middle of the range come out nearly white (~0.82 relative luminance)
# and lose contrast against a white page/background, which fails print/grayscale
# legibility (e.g. for RA-L). These two are built to have strictly decreasing
# luminance end-to-end (light -> dark) while keeping the same hue families
# (blue->red / teal->purple) so the two stay visually distinct from each other.
PASTEL_CMAP_SEQ = LinearSegmentedColormap.from_list(
    'pastel_sequential_blue_red',
    ['#96d1fa', '#a5c2fa', '#b0b6f5', '#b3abdc', '#c795db', '#d97ebf', '#e06c7f'],
)
PASTEL_CMAP_ALT_SEQ = LinearSegmentedColormap.from_list(
    'pastel_sequential_teal_purple',
    ['#82d8c7', '#8ec9d5', '#9eb8d8', '#a6a9cf', '#a499d1', '#a687ca', '#aa71bd'],
)

# ── Shared layout constants for figures meant to stack into one multi-panel
# figure (e.g. LinearSpring's force/stiffness-vs-k panels above a
# disturbance-response-vs-k panel). Keeping figure width, font size, and
# colorbar fraction/pad identical across figures makes their axes and
# colorbars line up when the panels are stacked vertically.
ALIGNED_FIG_W            = 9.0    # inches
ALIGNED_FONT_SIZE        = 28     # pt
ALIGNED_LEGEND_FONT_SIZE = ALIGNED_FONT_SIZE * 0.75
ALIGNED_LEFT             = 0.13   # axes left edge, fraction of figure width
ALIGNED_RIGHT            = 0.86   # axes right edge (before colorbar), fraction of figure width
ALIGNED_CBAR_FRACTION    = 0.06   # colorbar width, fraction of the axes it's attached to
ALIGNED_CBAR_PAD         = 0.02   # gap between axes and colorbar, fraction of figure width

# matplotlib's colorbar ties its width to its height via `aspect` (height =
# aspect * width, default aspect=20) and enforces that at draw/save time --
# so a colorbar spanning a taller region (e.g. 2 stacked panels) comes out
# visibly *thicker* than a shorter one even with identical fraction/pad. To
# hold the physical thickness constant across figures, each colorbar call
# should pass aspect=<its own vertical span in inches> / ALIGNED_CBAR_WIDTH_IN.
# Value = plot_disturbance.ipynb's single-panel colorbar span at the default
# aspect=20, i.e. the thickness that was already right there.
ALIGNED_CBAR_WIDTH_IN = ((0.94 - 0.18) * 5.2) / 20


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


def make_square_panel_row(n_panels, w_in, gap, left_margin, right_margin, bottom_margin, top_margin):
    """Lay out n_panels square (aspect='equal') panels left-to-right with a
    genuinely controllable physical gap between them, at a fixed total figure
    width.

    constrained_layout can't deliver this: when every panel is forced
    aspect='equal', each becomes a square whose side is fixed by the panel
    height, and if the requested total width is wider than N squares +
    margins, constrained_layout dumps ALL of that leftover width into the
    inter-panel gap regardless of the wspace/w_pad settings (measured
    ~1.08in gap even with both cranked to ~0, for N=2). This instead derives
    the square side from what's left after margins and the chosen gap, and
    places every axes by hand with fig.add_axes.

    Pass the same w_in as a companion figure (rather than matching per-panel
    width) to keep a fixed-pt font reading at the same size across both.

    All size arguments are in inches. Returns (fig, axes) -- axes is a list
    of length n_panels, left to right.
    """
    s = (w_in - left_margin - right_margin - (n_panels - 1) * gap) / n_panels
    h_in = s + bottom_margin + top_margin
    fig = plt.figure(figsize=(w_in, h_in))
    axes = [
        fig.add_axes([(left_margin + i * (s + gap)) / w_in, bottom_margin / h_in, s / w_in, s / h_in])
        for i in range(n_panels)
    ]
    return fig, axes


def fit_equal_scale(fig, axes, width, pad=0.06, gap=0.18):
    """Size `fig` and place `axes` so every axis of every panel has the SAME scale
    (inches per data unit): 10 cm along X, Y or Z is the same length on the page,
    in every panel, and a circle in the data is a circle on the page.

    Only `width` [in] is chosen. The height follows from the panels' data ranges
    -- panels stacked top to bottom, sharing one X range, each with its own Y
    range -- plus room for the text around them, which is measured, not guessed.
    Call it LAST: after plotting, limits, labels, ticks and the legend (a legend
    outside an axes is measured too). Panels are placed by hand and then held to
    aspect='equal'; nothing else may move them afterwards.

    pad: [in] clearance kept around the text; gap: [in] extra clearance between
    stacked panels (the upper one's X label above the lower one's frame).
    Returns the figure height [in].
    """
    axes = list(axes)
    fig.set_size_inches(width, 4.0)   # provisional: only the text extents are read from it
    total = None

    # The text clearance depends on the panels' final size (matplotlib picks fewer or more
    # ticks, and wider or narrower labels, as an axes grows), so measure again after each
    # placement until the layout stops changing -- a single pass can leave a label touching
    # the page edge.
    for _ in range(8):
        fig.canvas.draw()
        r, inv = fig.canvas.get_renderer(), fig.dpi_scale_trans.inverted()

        # Text/tick clearance outside each axes box [in]: left, right, below, above.
        dec = []
        for ax in axes:
            box = ax.get_window_extent(r).transformed(inv)
            tb  = ax.get_tightbbox(r).transformed(inv)
            dec.append((box.x0 - tb.x0, tb.x1 - box.x1, box.y0 - tb.y0, tb.y1 - box.y1))

        left   = max(d[0] for d in dec) + pad
        right  = max(d[1] for d in dec) + pad
        top    = dec[0][3] + pad
        bottom = dec[-1][2] + pad
        gaps   = [dec[i][2] + dec[i + 1][3] + gap for i in range(len(axes) - 1)]

        axes_w = width - left - right
        scale  = axes_w / abs(np.subtract(*axes[0].get_xlim()))   # in/cm, shared by every axis below
        heights = [scale * abs(np.subtract(*ax.get_ylim())) for ax in axes]
        new_total = top + sum(heights) + sum(gaps) + bottom

        fig.set_size_inches(width, new_total)
        y_top = new_total - top
        for k, (ax, h) in enumerate(zip(axes, heights)):
            ax.set_position([left / width, (y_top - h) / new_total, axes_w / width, h / new_total])
            ax.set_aspect('equal', adjustable='box')
            y_top -= h + (gaps[k] if k < len(gaps) else 0)

        settled = total is not None and abs(new_total - total) < 1e-4
        total = new_total
        if settled:
            break
    return total


def legend_figure(handles, labels, fontsize=None, ncol=1, pad=0.03):
    """A figure that holds nothing but a legend, sized to fit it -- for a plot that carries none.

    Pass `ax.get_legend_handles_labels()` of the plot it belongs to, so the two cannot disagree.
    The figure's size is the legend's own (set by `fontsize` and `ncol`) plus `pad` [in] on every
    side, so it is the same size on the page however it is saved, and can be placed at will.
    """
    fig = plt.figure(figsize=(2.0, 1.0))
    # Hosted in an invisible full-figure axes, not fig.legend(): IPython's inline backend
    # draws nothing for a figure without axes, and the legend would not show in the notebook.
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis('off')
    leg = ax.legend(handles, labels, loc='center', ncol=ncol, fontsize=fontsize,
                    frameon=False, borderpad=0, borderaxespad=0)
    fig.canvas.draw()
    box = leg.get_window_extent(fig.canvas.get_renderer()).transformed(fig.dpi_scale_trans.inverted())
    fig.set_size_inches(box.width + 2 * pad, box.height + 2 * pad)   # the legend re-centres on draw
    return fig


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
