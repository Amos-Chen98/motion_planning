#!/usr/bin/env python3

"""Plot six-dimensional joints_ctrl commands from rosbags."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import rosbag


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

# Manually edit these parameters before running the script.
PLACEHOLDER_BAG_PATH = (
    PACKAGE_DIR / "data" / "rosbag" / "2026-05-06-18-14-00_four_ring_success.bag"
)
BAG_PATHS = (
    PLACEHOLDER_BAG_PATH,
    PLACEHOLDER_BAG_PATH,
    PLACEHOLDER_BAG_PATH,
)
TOPIC = "/dragon/joints_ctrl"
DURATION_SECONDS = 60.0
OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "joint_cmd"
OUTPUT_BASENAME = "joint_cmd_three_runs"
SAVE_FIGURE = True
SHOW_PLOT = True
SAVE_DPI = 200
FIGURE_WIDTH_INCH = 3.5
FIGURE_HEIGHT_INCH = FIGURE_WIDTH_INCH * 9.0 / 16.0
FONT_SIZE_PT = 9
AXES_LEFT = 0.18
AXES_RIGHT = 0.965
AXES_BOTTOM = 0.2
AXES_TOP = 0.75
Y_LABEL_X = 0.075
LEGEND_ANCHOR_Y = 0.995
JOINT_COLORS = (
    "#4477AA",
    "#EE6677",
    "#228833",
    "#CCBB44",
    "#66CCEE",
    "#AA3377",
)

JOINT_LABELS = (
    "joint1 pitch",
    "joint1 yaw",
    "joint2 pitch",
    "joint2 yaw",
    "joint3 pitch",
    "joint3 yaw",
)


@dataclass(frozen=True)
class JointControlSeries:
    topic_names: tuple[str, ...]
    start_time: float
    requested_end_time: float
    actual_end_time: float
    times: list[float]
    positions: list[list[float]]
    exhausted_before_requested_end: bool


def normalized_topic_candidates(topic: str) -> tuple[str, ...]:
    stripped = topic.strip()
    if not stripped:
        raise RuntimeError("topic must not be empty")

    without_slash = stripped.lstrip("/")
    candidates = [stripped]
    if stripped.startswith("/"):
        candidates.append(without_slash)
    else:
        candidates.append("/" + stripped)

    unique_candidates = []
    for candidate in candidates:
        if candidate and candidate not in unique_candidates:
            unique_candidates.append(candidate)
    return tuple(unique_candidates)


def resolve_available_topics(bag: rosbag.Bag, requested_topic: str) -> tuple[str, ...]:
    available_topic_names = bag.get_type_and_topic_info().topics.keys()
    candidates = normalized_topic_candidates(requested_topic)
    resolved = tuple(candidate for candidate in candidates if candidate in available_topic_names)
    if resolved:
        return resolved

    raise RuntimeError(
        "topic not found in bag: {} (checked: {})".format(
            requested_topic,
            ", ".join(candidates),
        )
    )


def load_joint_control_series(
    bag_path: Path,
    requested_topic: str,
    duration: float,
) -> JointControlSeries:
    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError("bag file does not exist: {}".format(bag_path))
    if duration <= 0.0:
        raise RuntimeError("DURATION_SECONDS must be greater than 0")

    times: list[float] = []
    positions: list[list[float]] = []
    start_time = None
    requested_end_time = None
    topic_names: tuple[str, ...]
    exhausted_before_requested_end = True

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_names = resolve_available_topics(bag, requested_topic)
        for topic_name, msg, bag_time in bag.read_messages(topics=topic_names):
            absolute_time = bag_time.to_sec()
            if start_time is None:
                start_time = absolute_time
                requested_end_time = start_time + duration

            relative_time = absolute_time - start_time
            if relative_time > duration:
                exhausted_before_requested_end = False
                break

            position = list(msg.position)
            if len(position) != len(JOINT_LABELS):
                raise RuntimeError(
                    "{} at bag time {:.6f} has {} position values, expected {}".format(
                        topic_name,
                        absolute_time,
                        len(position),
                        len(JOINT_LABELS),
                    )
                )

            times.append(relative_time)
            positions.append(position)

    if start_time is None or requested_end_time is None:
        raise RuntimeError("no messages found for topic: {}".format(requested_topic))

    return JointControlSeries(
        topic_names=topic_names,
        start_time=start_time,
        requested_end_time=requested_end_time,
        actual_end_time=start_time + times[-1],
        times=times,
        positions=positions,
        exhausted_before_requested_end=exhausted_before_requested_end,
    )


def import_pyplot(use_noninteractive_backend: bool):
    if use_noninteractive_backend:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    return plt


def configure_plot_style(plt) -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman"],
            "font.size": FONT_SIZE_PT,
            "axes.labelsize": FONT_SIZE_PT,
            "axes.titlesize": FONT_SIZE_PT,
            "axes.facecolor": "white",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "xtick.labelsize": FONT_SIZE_PT,
            "ytick.labelsize": FONT_SIZE_PT,
            "legend.fontsize": FONT_SIZE_PT,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
        }
    )


def plot_joint_control_series(
    series_list: tuple[JointControlSeries, ...],
    duration: float,
    plt,
):
    configure_plot_style(plt)

    fig, axes = plt.subplots(
        len(series_list),
        1,
        sharex=True,
        figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH),
    )
    if len(series_list) == 1:
        axes = (axes,)

    fig.subplots_adjust(
        left=AXES_LEFT,
        right=AXES_RIGHT,
        bottom=AXES_BOTTOM,
        top=AXES_TOP,
        hspace=0.18,
    )
    fig.text(
        Y_LABEL_X,
        0.47,
        "Joint command [rad]",
        rotation="vertical",
        ha="center",
        va="center",
        fontsize=FONT_SIZE_PT,
    )

    linestyles = ["-", "-", "-", "-", "-", "-"]

    for ax, series in zip(axes, series_list):
        series_by_joint = list(zip(*series.positions))
        for values, label, color, linestyle in zip(
            series_by_joint,
            JOINT_LABELS,
            JOINT_COLORS,
            linestyles,
        ):
            ax.plot(
                series.times,
                values,
                label=label,
                color=color,
                linestyle=linestyle,
                linewidth=1.0,
            )

        ax.axhline(0.0, color="0.35", linewidth=0.55, alpha=0.55)
        ax.grid(True, which="major", color="0.82", linewidth=0.4, alpha=0.7)
        ax.set_axisbelow(True)

    x_max = max(
        series.times[-1] if series.exhausted_before_requested_end else duration
        for series in series_list
    )
    if x_max <= 0.0:
        x_max = 1.0

    axes[-1].set_xlim(0.0, x_max)
    axes[-1].set_xlabel("Time [s]")

    handles, labels = axes[0].get_legend_handles_labels()
    legend_order = [0, 1, 2, 3, 4, 5]
    fig.legend(
        [handles[index] for index in legend_order],
        [labels[index] for index in legend_order],
        loc="upper center",
        bbox_to_anchor=((AXES_LEFT + AXES_RIGHT) * 0.5, LEGEND_ANCHOR_Y),
        ncol=3,
        frameon=True,
        framealpha=0.95,
        borderpad=0.3,
        labelspacing=0.2,
        handlelength=1.2,
        handletextpad=0.35,
        columnspacing=0.65,
    )
    return fig


def resolve_output_svg_path(output_dir: Path, output_basename: str) -> Path:
    directory = Path(output_dir).expanduser().resolve()
    return directory / "{}.svg".format(output_basename)


def print_summary(series: JointControlSeries, duration: float) -> None:
    actual_duration = series.actual_end_time - series.start_time
    print(
        "Read {} messages from {}.".format(
            len(series.times),
            ", ".join(series.topic_names),
        )
    )
    print(
        "Plot window: bag time {:.6f} -> {:.6f}; relative {:.3f} s -> {:.3f} s.".format(
            series.start_time,
            series.actual_end_time,
            0.0,
            actual_duration,
        )
    )
    print(
        "Requested end: {:.3f} s after first message (bag time {:.6f}).".format(
            duration,
            series.requested_end_time,
        )
    )
    if series.exhausted_before_requested_end:
        print("Requested duration exceeds available topic data; plotted through the final message.")


def main() -> int:
    try:
        series_list = tuple(
            load_joint_control_series(bag_path, TOPIC, DURATION_SECONDS)
            for bag_path in BAG_PATHS
        )
        for bag_path, series in zip(BAG_PATHS, series_list):
            print("Bag: {}".format(Path(bag_path).expanduser().resolve()))
            print_summary(series, DURATION_SECONDS)

        plt = import_pyplot(not SHOW_PLOT)
        fig = plot_joint_control_series(series_list, DURATION_SECONDS, plt)

        if SAVE_FIGURE:
            output_path = resolve_output_svg_path(OUTPUT_DIR, OUTPUT_BASENAME)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(output_path, dpi=SAVE_DPI)
            print("Saved plot: {}".format(output_path))

        if SHOW_PLOT:
            plt.show()

        plt.close(fig)
        return 0
    except (RuntimeError, rosbag.ROSBagException) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
