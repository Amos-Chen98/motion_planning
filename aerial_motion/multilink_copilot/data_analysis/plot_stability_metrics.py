#!/usr/bin/env python3

"""Plot stability metric time series from a rosbag."""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

import rosbag


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

# Manually edit these parameters before running the script.
BAG_PATH = (
    PACKAGE_DIR
    / "data"
    / "rosbag"
    / "stability_metrics"
    / "root_circular_trajectory_2026-05-23-15-22-07.bag"
)
OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "stability_metrics"
SAVE_FIGURE = True
SHOW_PLOT = True
OUTPUT_FORMATS = ("pdf", "svg")
SAVE_DPI = 200
# IEEE two-column text width; a quarter-page figure occupies 1/4 of it so that
# four such figures fit across the double-column page width.
IEEE_PAGE_WIDTH_INCH = 7.16
FIGURE_WIDTH_INCH = IEEE_PAGE_WIDTH_INCH / 4.0
FIGURE_HEIGHT_INCH = FIGURE_WIDTH_INCH * 3.0 / 4.0
FONT_SIZE_PT = 7
LEGEND_FONT_SIZE_PT = 5.5
# The y-axis label is the terse symbol + unit, so the left margin only needs to
# clear the tick labels and the rotated symbol; this leaves more axes width for
# the descriptive legend below. It is kept wide enough for the four-character
# tick labels (e.g. "0.40") of the clearance panel.
AXES_LEFT = 0.24
AXES_RIGHT = 0.985
AXES_BOTTOM = 0.25
AXES_TOP = 0.95
METRIC_COLOR = "#0072B2"
THRESHOLD_COLOR = "#D55E00"
Y_AXIS_BOTTOM_DATA_MAX_FRACTION = 0.8
Y_AXIS_TOP_MARGIN_FRACTION = 0.06

FC_RP_MIN_THRESHOLD = 3.0
OVERLAP_CLEARANCE_THRESHOLD = 0.3


@dataclass(frozen=True)
class MetricConfig:
    topic: str
    ylabel: str
    legend_label: str
    threshold_label: str
    threshold: float
    output_suffix: str


@dataclass(frozen=True)
class MetricSeries:
    config: MetricConfig
    topic_names: tuple[str, ...]
    start_time: float
    actual_end_time: float
    times: tuple[float, ...]
    values: tuple[float, ...]


METRICS = (
    MetricConfig(
        topic="/dragon/stability/fc_rp_min",
        ylabel=r"$d_{\mathrm{rp}}$ [\si{\newton\meter}]",
        # Symbols follow Table~\ref{tab:copilot_parameters}: $d_{\mathrm{rp}}$ is
        # the roll/pitch feasible-control margin and $\underline{d}_{\mathrm{rp}}$
        # is its lower bound. Each legend entry pairs a plain-language name with
        # the symbol so the figure is readable without the table.
        legend_label=r"Feasible-control margin $d_{\mathrm{rp}}$",
        threshold_label=(
            r"Lower bound $\underline{d}_{\mathrm{rp}}="
            rf"\SI{{{FC_RP_MIN_THRESHOLD}}}{{\newton\meter}}$"
        ),
        threshold=FC_RP_MIN_THRESHOLD,
        output_suffix="fc_rp_min",
    ),
    MetricConfig(
        topic="/dragon/stability/overlap_clearance",
        ylabel=r"$c$ [\si{\meter}]",
        # $c$ is the rotor clearance and $\underline{c}$ its threshold, matching
        # Table~\ref{tab:copilot_parameters}.
        legend_label=r"Rotor clearance $c$",
        threshold_label=(
            r"Clearance threshold $\underline{c}="
            rf"\SI{{{OVERLAP_CLEARANCE_THRESHOLD}}}{{\meter}}$"
        ),
        threshold=OVERLAP_CLEARANCE_THRESHOLD,
        output_suffix="overlap_clearance",
    ),
)


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


def load_metric_series(bag_path: Path, config: MetricConfig) -> MetricSeries:
    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError("bag file does not exist: {}".format(bag_path))

    times: list[float] = []
    values: list[float] = []
    start_time = None
    actual_end_time = None
    topic_names: tuple[str, ...]

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_names = resolve_available_topics(bag, config.topic)
        for topic_name, msg, bag_time in bag.read_messages(topics=topic_names):
            absolute_time = bag_time.to_sec()
            if start_time is None:
                start_time = absolute_time

            value = float(msg.data)
            if not math.isfinite(value):
                raise RuntimeError(
                    "{} at bag time {:.6f} has non-finite value {}".format(
                        topic_name,
                        absolute_time,
                        value,
                    )
                )

            times.append(absolute_time - start_time)
            values.append(value)
            actual_end_time = absolute_time

    if start_time is None or actual_end_time is None:
        raise RuntimeError("no messages found for topic: {}".format(config.topic))

    return MetricSeries(
        config=config,
        topic_names=topic_names,
        start_time=start_time,
        actual_end_time=actual_end_time,
        times=tuple(times),
        values=tuple(values),
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
            "font.serif": [
                "Nimbus Roman",
                "Nimbus Roman No9 L",
                "Times New Roman",
                "Times",
            ],
            "font.size": FONT_SIZE_PT,
            "axes.labelsize": FONT_SIZE_PT,
            "axes.titlesize": FONT_SIZE_PT,
            "axes.facecolor": "white",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "xtick.labelsize": FONT_SIZE_PT,
            "ytick.labelsize": FONT_SIZE_PT,
            "legend.fontsize": LEGEND_FONT_SIZE_PT,
            "text.usetex": True,
            "text.latex.preamble": "\n".join(
                (
                    r"\usepackage{newtxtext}",
                    r"\usepackage{newtxmath}",
                    r"\usepackage{siunitx}",
                    r"\sisetup{detect-weight=true, detect-family=true}",
                    r"\sisetup{per-mode=symbol}",
                    r"\sisetup{inter-unit-product = \mathord{\cdot}}",
                )
            ),
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "path",
        }
    )


def metric_y_limits(values: tuple[float, ...], threshold: float) -> tuple[float, float]:
    data_max = max(values)
    display_max = max(data_max, threshold)
    if display_max <= 0.0:
        display_max = 1.0

    y_min = data_max * Y_AXIS_BOTTOM_DATA_MAX_FRACTION
    y_max = display_max * (1.0 + Y_AXIS_TOP_MARGIN_FRACTION)
    return y_min, y_max


def plot_metric_series(series: MetricSeries, plt):
    configure_plot_style(plt)

    fig, ax = plt.subplots(figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH))
    fig.subplots_adjust(
        left=AXES_LEFT,
        right=AXES_RIGHT,
        bottom=AXES_BOTTOM,
        top=AXES_TOP,
    )

    ax.plot(
        series.times,
        series.values,
        color=METRIC_COLOR,
        linestyle="-",
        linewidth=1.2,
        label=series.config.legend_label,
    )
    ax.axhline(
        series.config.threshold,
        color=THRESHOLD_COLOR,
        linestyle="--",
        linewidth=1.0,
        alpha=0.9,
        label=series.config.threshold_label,
    )

    x_max = series.times[-1]
    if x_max <= 0.0:
        x_max = 1.0

    ax.set_xlim(0.0, x_max)
    ax.set_ylim(*metric_y_limits(series.values, series.config.threshold))
    ax.set_xlabel(r"$t$ [\si{\second}]", labelpad=1.0)
    ax.set_ylabel(series.config.ylabel)
    ax.tick_params(axis="both", which="major", pad=1.0, length=2.0, width=0.6)
    ax.legend(
        loc="lower center",
        ncol=1,
        frameon=True,
        framealpha=0.85,
        edgecolor="0.75",
        borderpad=0.2,
        handlelength=1.0,
        handletextpad=0.35,
        columnspacing=0.65,
        labelspacing=0.2,
    )
    ax.grid(True, which="major", color="0.82", linewidth=0.45, alpha=0.7)
    ax.set_axisbelow(True)
    return fig


def resolve_output_path(
    output_dir: Path, bag_path: Path, config: MetricConfig, output_format: str
) -> Path:
    directory = Path(output_dir).expanduser().resolve()
    return directory / "{}_{}.{}".format(
        Path(bag_path).stem,
        config.output_suffix,
        output_format,
    )


def print_summary(series: MetricSeries) -> None:
    actual_duration = series.actual_end_time - series.start_time
    values = series.values
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
        "{}: min={:.6f}, mean={:.6f}, max={:.6f}, threshold={:.6f}".format(
            series.config.ylabel,
            min(values),
            statistics.fmean(values),
            max(values),
            series.config.threshold,
        )
    )


def main() -> int:
    try:
        plt = import_pyplot(not SHOW_PLOT)
        figures = []

        for config in METRICS:
            series = load_metric_series(BAG_PATH, config)
            print_summary(series)

            fig = plot_metric_series(series, plt)
            figures.append(fig)

            if SAVE_FIGURE:
                for output_format in OUTPUT_FORMATS:
                    output_path = resolve_output_path(
                        OUTPUT_DIR,
                        BAG_PATH,
                        config,
                        output_format,
                    )
                    output_path.parent.mkdir(parents=True, exist_ok=True)
                    fig.savefig(output_path, dpi=SAVE_DPI, format=output_format)
                    print("Saved plot: {}".format(output_path))

        if SHOW_PLOT:
            plt.show()

        for fig in figures:
            plt.close(fig)
        return 0
    except (RuntimeError, rosbag.ROSBagException) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
