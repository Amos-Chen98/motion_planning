#!/usr/bin/env python3

"""Plot subsequent-link tracking separation for one rosbag scene.

This is the single-scenario counterpart of ``plot_link_tracking_separation.py``.
It reads the shared-control rosbag directly, computes each subsequent link's
tail-crossing separation from link 1, and saves a compact dot plot.
"""

from __future__ import annotations

import argparse
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rosbag
import rospy
import tf2_ros

from link_tail_separation_evaluator import LinkTailSeparationEvaluator


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
BAG_PATH = (
    PACKAGE_DIR
    / "data"
    / "rosbag"
    / "2026-05-28-17-06-21_shared_control_success.bag"
)
OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "separation"
OUTPUT_STEM = "shared_control_link_tracking_separation_dotplot"

# IEEE Transactions two-column text width; a half-page figure spans one column.
IEEE_PAGE_WIDTH_INCH = 7.16
FIGURE_WIDTH_INCH = IEEE_PAGE_WIDTH_INCH / 2.0
FIGURE_HEIGHT_INCH = 1.76
FONT_SIZE_PT = 9
SAVE_DPI = 200
SAVEFIG_KWARGS = {
    "bbox_inches": "tight",
    "pad_inches": 0.0,
}

LINK_INDICES = (2, 3, 4)
RING_RADIUS_COLOR = "0.45"
SCENE_COLOR = "#7F2EA8"
SCENE_MARKER = "D"
SCENE_LABEL = r"(d) Teleoperation"
POINT_JITTER_HALF_WIDTH = 0.035
JITTER_SEED = 0
SUMMARY_OFFSET = 0.16


@dataclass(frozen=True)
class LinkSamples:
    """Separations for one subsequent link in a single bag."""

    link_index: int
    values_m: tuple[float, ...]
    mean_m: float
    min_m: float
    max_m: float


class InMemoryLinkTailSeparationEvaluator(LinkTailSeparationEvaluator):
    """Run the existing separation evaluator without writing a JSON file."""

    def __init__(self, bag_path: Path):
        bag_path = Path(bag_path).expanduser().resolve()
        super().__init__(bag_path.stem)
        self.bag_path = str(bag_path)

    def run_in_memory(self) -> dict:
        if not Path(self.bag_path).is_file():
            raise RuntimeError("bag file does not exist: {}".format(self.bag_path))

        with rosbag.Bag(self.bag_path) as bag:
            topic_infos = bag.get_type_and_topic_info().topics
            self.root_pose_topic = self.resolve_topic(
                topic_infos, "dragon/root/pose", required=True
            )
            self.tf_topic = self.resolve_topic(topic_infos, "/tf", required=True)
            self.tf_static_topic = self.resolve_topic(
                topic_infos, "/tf_static", required=False
            )
            self.waypoints_by_index = self.discover_waypoints(topic_infos)

            if not self.waypoints_by_index:
                raise RuntimeError("no waypoint pose topics found in {}".format(self.bag_path))

            cache_time = (bag.get_end_time() - bag.get_start_time()) + 30.0
            self.tf_buffer = tf2_ros.Buffer(
                cache_time=rospy.Duration.from_sec(cache_time),
                debug=False,
            )

            self.load_waypoints_and_tf(bag)
            self.process_root_poses(bag)
            return self.build_result(bag)


def load_samples_from_bag(bag_path: Path) -> tuple[list[LinkSamples], float, dict]:
    result = InMemoryLinkTailSeparationEvaluator(bag_path).run_in_memory()
    samples: list[LinkSamples] = []

    for link_index in LINK_INDICES:
        key = "link{}_tail".format(link_index)
        values = [
            float(waypoint["separation_from_link1_m"][key])
            for waypoint in result["waypoints"]
            if waypoint.get("status") == "complete"
            and waypoint["separation_from_link1_m"].get(key) is not None
        ]
        if not values:
            raise RuntimeError("no complete crossings found for {}".format(key))

        samples.append(
            LinkSamples(
                link_index=link_index,
                values_m=tuple(values),
                mean_m=statistics.fmean(values),
                min_m=min(values),
                max_m=max(values),
            )
        )

    return samples, float(result["ring_radius_m"]), result


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


def build_figure(samples: list[LinkSamples], ring_radius: float, plt):
    configure_plot_style(plt)
    rng = np.random.default_rng(JITTER_SEED)

    fig, ax = plt.subplots(figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH))
    fig.subplots_adjust(left=0.165, right=0.975, bottom=0.205, top=0.965)

    legend_seen: set[str] = set()
    for position, sample in enumerate(samples):
        jitter = rng.uniform(
            -POINT_JITTER_HALF_WIDTH,
            POINT_JITTER_HALF_WIDTH,
            size=len(sample.values_m),
        )
        xs = position - SUMMARY_OFFSET + jitter
        ax.scatter(
            xs,
            sample.values_m,
            s=24,
            color=SCENE_COLOR,
            marker=SCENE_MARKER,
            alpha=0.9,
            edgecolors="white",
            linewidths=0.4,
            zorder=3,
            label=SCENE_LABEL if "scene" not in legend_seen else None,
        )
        legend_seen.add("scene")

        ax.errorbar(
            position + SUMMARY_OFFSET,
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

    data_max = max(sample.max_m for sample in samples)
    ax.set_xlim(-0.55, len(samples) - 0.35)
    ax.set_ylim(-0.008, max(ring_radius, data_max) * 1.12)
    ax.set_xticks(range(len(samples)))
    ax.set_xticklabels(["Link {}".format(sample.link_index) for sample in samples])
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


def print_summary(samples: list[LinkSamples], result: dict) -> None:
    summary = result["summary"]
    print("bag: {}".format(result["bag_name"]))
    print(
        "complete waypoints: {}/{}".format(
            summary["complete_waypoint_count"],
            summary["waypoint_count"],
        )
    )
    for sample in samples:
        print(
            "link {}: n={}, mean={:.4f} m, min={:.4f} m, max={:.4f} m".format(
                sample.link_index,
                len(sample.values_m),
                sample.mean_m,
                sample.min_m,
                sample.max_m,
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-path", type=Path, default=BAG_PATH)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    try:
        samples, ring_radius, result = load_samples_from_bag(args.bag_path)
        print_summary(samples, result)

        if args.show:
            import matplotlib.pyplot as plt
        else:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

        fig = build_figure(samples, ring_radius, plt)

        args.output_dir.mkdir(parents=True, exist_ok=True)
        svg_path = args.output_dir / "{}.svg".format(OUTPUT_STEM)
        fig.savefig(svg_path, dpi=SAVE_DPI, format="svg", **SAVEFIG_KWARGS)
        print("Saved figure: {}".format(svg_path))

        if args.show:
            plt.show()
        plt.close(fig)
        return 0
    except (RuntimeError, ValueError, rosbag.ROSBagException) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
