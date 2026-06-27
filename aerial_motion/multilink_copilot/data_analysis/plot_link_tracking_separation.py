#!/usr/bin/env python3

"""Dot-plot of subsequent-link tracking error through the ring waypoints.

Each accepted ring crossing contributes one point per subsequent link, namely
the separation between that link's tail crossing position and the root-link
(link 1) tail crossing position. Small, bounded separation that grows mildly
with link index is the expected follow-the-leader behavior of the copilot.

For each subsequent link (2, 3, 4) the figure overlays:
  * jittered raw points, one per ring crossing, coloured by ring arrangement,
  * a black marker at the mean with whiskers spanning the [min, max] range.

The raw points are shown because the per-link sample size is small (one point
per ring crossing), so a kernel-density form (e.g. a violin) would fabricate a
distribution shape the data does not support; a dot-plot stays honest.

The figure is sized for an IEEE Transactions two-column layout: it occupies one
half of the text width and renders all text at 9 pt via ``usetex`` with the
Times-compatible newtx fonts, matching the manuscript body font. The saved
figure uses tight cropping with zero padding so the axis text sits directly at
the image boundary.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
DATA_DIR = PACKAGE_DIR / "data" / "separation"
OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "separation"
OUTPUT_STEM = "link_tracking_separation_dotplot"

# IEEE Transactions two-column text width; a half-page figure spans one column.
IEEE_PAGE_WIDTH_INCH = 7.16
FIGURE_WIDTH_INCH = IEEE_PAGE_WIDTH_INCH / 2.0
FIGURE_HEIGHT_INCH = 1.76  # 3/4 of the earlier 2.35 in height
FONT_SIZE_PT = 9
SAVE_DPI = 200
SAVEFIG_KWARGS = {
    "bbox_inches": "tight",
    "pad_inches": 0.0,
}

# Subsequent links that have a separation-from-link-1 entry in the JSON.
LINK_INDICES = (2, 3, 4)

# Within each link group the scenarios occupy separate side-by-side sub-columns
# (one column per arrangement) so their points do not overlap, with the pooled
# summary marker in the rightmost slot. COLUMN_SPREAD is the total x-width these
# slots span around the tick; a tiny jitter declutters repeated points within a
# column.
COLUMN_SPREAD = 0.60
POINT_JITTER_HALF_WIDTH = 0.10  # spread within the 0.20 slot spacing; columns stay distinct
JITTER_SEED = 0

RING_RADIUS_COLOR = "0.45"


@dataclass(frozen=True)
class Arrangement:
    """A ring-traversal run mapped to its manuscript label and plot style."""

    key: str  # substring identifying the JSON file
    label: str
    color: str
    marker: str


# Planner-driven arrangements, ordered and labelled as in Fig.~\ref{fig:wpt_exp}.
# Colours follow the Okabe--Ito palette already used in the stability figures.
PLANNER_ARRANGEMENTS = (
    Arrangement("close_ring", r"(a) Three vertical rings", "#0072B2", "o"),
    Arrangement("large_pitch", r"(b) Vertical \& horizontal", "#D55E00", "s"),
    Arrangement("four_ring", r"(c) Four sequential rings", "#009E73", "^"),
)
# Teleoperation is a separate scenario (single crossing); excluded by default so
# it does not skew the planner-run mean and range.
TELEOP_ARRANGEMENT = Arrangement(
    "shared_control", r"(d) Teleoperation", "#7F2EA8", "D"
)


@dataclass(frozen=True)
class LinkSamples:
    """Per-link separations grouped by arrangement, plus the pooled summary."""

    link_index: int
    by_arrangement: dict[str, tuple[float, ...]]
    mean_m: float
    min_m: float
    max_m: float


def find_json_for(arrangement: Arrangement, data_dir: Path) -> Path:
    matches = sorted(p for p in data_dir.glob("*.json") if arrangement.key in p.name)
    if not matches:
        raise RuntimeError(
            "no JSON file containing '{}' in {}".format(arrangement.key, data_dir)
        )
    if len(matches) > 1:
        raise RuntimeError(
            "ambiguous JSON files for '{}': {}".format(
                arrangement.key, ", ".join(p.name for p in matches)
            )
        )
    return matches[0]


def load_separations(path: Path) -> tuple[dict[int, list[float]], float]:
    """Return per-link separation lists and the ring radius from one JSON file."""

    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)

    ring_radius = float(payload["ring_radius_m"])
    per_link: dict[int, list[float]] = {link: [] for link in LINK_INDICES}
    for waypoint in payload["waypoints"]:
        if waypoint.get("status") != "complete":
            continue
        separations = waypoint["separation_from_link1_m"]
        for link in LINK_INDICES:
            value = separations.get("link{}_tail".format(link))
            if value is None:
                raise RuntimeError(
                    "waypoint {} in {} lacks link{}_tail separation".format(
                        waypoint.get("index"), path.name, link
                    )
                )
            per_link[link].append(float(value))
    return per_link, ring_radius


def collect_samples(
    arrangements: tuple[Arrangement, ...], data_dir: Path
) -> tuple[list[LinkSamples], float]:
    raw: dict[int, dict[str, tuple[float, ...]]] = {
        link: {} for link in LINK_INDICES
    }
    ring_radii: list[float] = []
    for arrangement in arrangements:
        per_link, ring_radius = load_separations(find_json_for(arrangement, data_dir))
        ring_radii.append(ring_radius)
        for link in LINK_INDICES:
            raw[link][arrangement.key] = tuple(per_link[link])

    if max(ring_radii) - min(ring_radii) > 1e-9:
        raise RuntimeError("ring_radius_m differs across runs: {}".format(ring_radii))

    samples: list[LinkSamples] = []
    for link in LINK_INDICES:
        pooled = [v for values in raw[link].values() for v in values]
        if not pooled:
            raise RuntimeError("no crossings found for link {}".format(link))
        samples.append(
            LinkSamples(
                link_index=link,
                by_arrangement=raw[link],
                mean_m=float(np.mean(pooled)),
                min_m=float(np.min(pooled)),
                max_m=float(np.max(pooled)),
            )
        )
    return samples, ring_radii[0]


def configure_plot_style(plt) -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": [
                "Nimbus Roman",
                "Nimbus Roman No9 L",
                "Times New Roman",
                "Times",
            ],
            "font.size": FONT_SIZE_PT,
            "axes.labelsize": FONT_SIZE_PT,
            "axes.titlesize": FONT_SIZE_PT,
            "xtick.labelsize": FONT_SIZE_PT,
            "ytick.labelsize": FONT_SIZE_PT,
            "legend.fontsize": FONT_SIZE_PT,
            "axes.facecolor": "white",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "text.usetex": True,
            "text.latex.preamble": "\n".join(
                (
                    r"\usepackage{newtxtext}",
                    r"\usepackage{newtxmath}",
                    r"\usepackage{siunitx}",
                    r"\sisetup{detect-weight=true, detect-family=true}",
                )
            ),
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "path",
        }
    )


def build_figure(
    samples: list[LinkSamples],
    arrangements: tuple[Arrangement, ...],
    ring_radius: float,
    plt,
):
    configure_plot_style(plt)
    style_by_key = {a.key: a for a in arrangements}
    rng = np.random.default_rng(JITTER_SEED)

    fig, ax = plt.subplots(figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH))
    fig.subplots_adjust(left=0.165, right=0.975, bottom=0.205, top=0.965)

    # One sub-column per arrangement, with the summary marker in the last slot.
    slot_offsets = np.linspace(
        -COLUMN_SPREAD / 2.0, COLUMN_SPREAD / 2.0, len(arrangements) + 1
    )

    legend_seen: set[str] = set()
    for position, sample in enumerate(samples):
        for index, arrangement in enumerate(arrangements):
            values = sample.by_arrangement.get(arrangement.key, ())
            if not values:
                continue
            jitter = rng.uniform(
                -POINT_JITTER_HALF_WIDTH, POINT_JITTER_HALF_WIDTH, size=len(values)
            )
            xs = position + slot_offsets[index] + jitter
            ax.scatter(
                xs,
                values,
                s=22,
                color=arrangement.color,
                marker=arrangement.marker,
                alpha=0.9,
                edgecolors="white",
                linewidths=0.4,
                zorder=3,
                label=arrangement.label if arrangement.key not in legend_seen else None,
            )
            legend_seen.add(arrangement.key)

        ax.errorbar(
            position + slot_offsets[-1],
            sample.mean_m,
            yerr=[[sample.mean_m - sample.min_m], [sample.max_m - sample.mean_m]],
            fmt="D",
            color="black",
            markersize=4,
            markerfacecolor="black",
            markeredgecolor="black",
            capsize=3,
            elinewidth=1.1,
            capthick=1.1,
            zorder=4,
            label=r"Mean $\pm$ range" if "summary" not in legend_seen else None,
        )
        legend_seen.add("summary")

    ax.axhline(ring_radius, color=RING_RADIUS_COLOR, linestyle=":", linewidth=1.0)
    ax.text(
        len(samples) - 0.55,
        ring_radius + 0.012,
        r"ring radius (\SI{0.4}{\meter})",
        ha="right",
        va="bottom",
        color=RING_RADIUS_COLOR,
        fontsize=FONT_SIZE_PT,
    )

    data_max = max(s.max_m for s in samples)
    ax.set_xlim(-0.55, len(samples) - 0.35)
    ax.set_ylim(-0.008, max(ring_radius, data_max) * 1.12)
    ax.set_xticks(range(len(samples)))
    ax.set_xticklabels(["Link {}".format(s.link_index) for s in samples])
    ax.set_xlabel("Subsequent link", labelpad=2.0)
    ax.set_ylabel(
        "Crossing separation\n" r"from link\,1 [\si{\meter}]", labelpad=2.0
    )
    ax.tick_params(axis="both", which="major", pad=2.0, length=2.5, width=0.6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.yaxis.grid(True, color="0.85", linewidth=0.5, alpha=0.8)
    ax.set_axisbelow(True)
    ax.legend(
        loc="upper left",
        frameon=True,
        framealpha=0.9,
        edgecolor="0.8",
        borderpad=0.3,
        handlelength=1.2,
        handletextpad=0.4,
        labelspacing=0.25,
    )
    return fig


def print_summary(samples: list[LinkSamples]) -> None:
    for sample in samples:
        count = sum(len(v) for v in sample.by_arrangement.values())
        print(
            "link {}: n={}, mean={:.4f} m, min={:.4f} m, max={:.4f} m".format(
                sample.link_index, count, sample.mean_m, sample.min_m, sample.max_m
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument(
        "--include-teleop",
        action="store_true",
        help="also plot the teleoperation run as arrangement (d)",
    )
    parser.add_argument(
        "--preview-png",
        type=Path,
        default=None,
        help="optional path to also save a raster preview for inspection",
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    arrangements = PLANNER_ARRANGEMENTS
    if args.include_teleop:
        arrangements = arrangements + (TELEOP_ARRANGEMENT,)

    try:
        samples, ring_radius = collect_samples(arrangements, args.data_dir)
        print_summary(samples)

        if args.show:
            import matplotlib.pyplot as plt
        else:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

        fig = build_figure(samples, arrangements, ring_radius, plt)

        args.output_dir.mkdir(parents=True, exist_ok=True)
        svg_path = args.output_dir / "{}.svg".format(OUTPUT_STEM)
        fig.savefig(svg_path, dpi=SAVE_DPI, format="svg", **SAVEFIG_KWARGS)
        print("Saved figure: {}".format(svg_path))

        if args.preview_png is not None:
            args.preview_png.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(args.preview_png, dpi=SAVE_DPI, format="png", **SAVEFIG_KWARGS)
            print("Saved preview: {}".format(args.preview_png))

        if args.show:
            plt.show()
        plt.close(fig)
        return 0
    except RuntimeError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
