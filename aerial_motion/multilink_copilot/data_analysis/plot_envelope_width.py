#!/usr/bin/env python3

"""Plot a scatter plot of waypoint envelope widths from batch JSON results."""

from __future__ import annotations

import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

# Manually edit these parameters before running the script.
DATA_ROOT = (
    PACKAGE_DIR
    / "data"
    / "envelope_width"
    / "waypoint_conditioned_batch"
    / "20260509172527"
)
OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "envelope_width"
OUTPUT_FILENAME = "waypoint_conditioned_envelope_width_scatter.pdf"
EXPECTED_COMPLETED_WAYPOINTS = 4
MAX_VALID_JSON_FILES = 100
SAVE_FIGURE = False
SHOW_PLOT = True
SAVE_DPI = 200
FIGURE_WIDTH_INCH = 3.5
FIGURE_HEIGHT_INCH = 2.45
FONT_SIZE_PT = 9
SCATTER_COLOR = "#0072B2"
SCATTER_EDGE_COLOR = "white"
SCATTER_MARKER_SIZE_PT2 = 18
SCATTER_ALPHA = 0.78
SCATTER_EDGE_LINEWIDTH = 0.35
SCATTER_CENTER_X = 0.0
SCATTER_JITTER_HALF_WIDTH = 0.28


@dataclass(frozen=True)
class EnvelopeWidthSamples:
    successful_files: tuple[Path, ...]
    failed_files: tuple[Path, ...]
    invalid_files: tuple[Path, ...]
    unprocessed_file_count: int
    widths_m: tuple[float, ...]


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
        }
    )


def load_json_file(json_path: Path) -> dict[str, Any]:
    try:
        with json_path.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except json.JSONDecodeError as exc:
        raise RuntimeError("failed to parse JSON file {}: {}".format(json_path, exc)) from exc
    except OSError as exc:
        raise RuntimeError("failed to read JSON file {}: {}".format(json_path, exc)) from exc

    if not isinstance(data, dict):
        raise RuntimeError("JSON root must be an object: {}".format(json_path))
    return data


def completed_waypoint_count(data: dict[str, Any], json_path: Path) -> int:
    summary = data.get("summary")
    if not isinstance(summary, dict):
        raise RuntimeError("missing summary object in {}".format(json_path))

    count = summary.get("completed_waypoint_count")
    if not isinstance(count, int) or isinstance(count, bool):
        raise RuntimeError("invalid completed_waypoint_count in {}".format(json_path))
    return count


def envelope_widths_from_completed_file(data: dict[str, Any], json_path: Path) -> tuple[float, ...]:
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list):
        raise RuntimeError("missing waypoints array in {}".format(json_path))

    widths: list[float] = []
    for waypoint_index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise RuntimeError(
                "waypoint {} in {} must be an object".format(waypoint_index, json_path)
            )

        value = waypoint.get("envelope_width_m")
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise RuntimeError(
                "waypoint {} in {} has non-numeric envelope_width_m".format(
                    waypoint_index,
                    json_path,
                )
            )
        if not math.isfinite(float(value)):
            raise RuntimeError(
                "waypoint {} in {} has non-finite envelope_width_m".format(
                    waypoint_index,
                    json_path,
                )
            )

        widths.append(float(value))

    if len(widths) != EXPECTED_COMPLETED_WAYPOINTS:
        raise RuntimeError(
            "{} produced {} envelope widths, expected {}".format(
                json_path,
                len(widths),
                EXPECTED_COMPLETED_WAYPOINTS,
            )
        )
    return tuple(widths)


def case_id_from_json_path(json_path: Path) -> str:
    for path_part in reversed(json_path.parts):
        if path_part.startswith("case_"):
            return path_part
    return json_path.parent.name


def load_envelope_width_samples(
    data_root: Path,
    max_valid_json_files: int,
) -> EnvelopeWidthSamples:
    data_root = Path(data_root).expanduser().resolve()
    if not data_root.is_dir():
        raise RuntimeError("data root does not exist: {}".format(data_root))
    if max_valid_json_files <= 0:
        raise RuntimeError("MAX_VALID_JSON_FILES must be greater than 0")

    json_paths = tuple(sorted(data_root.rglob("*.json")))
    if not json_paths:
        raise RuntimeError("no JSON files found under {}".format(data_root))

    successful_files: list[Path] = []
    failed_files: list[Path] = []
    invalid_files: list[Path] = []
    widths_m: list[float] = []
    processed_file_count = 0

    for json_index, json_path in enumerate(json_paths):
        processed_file_count = json_index + 1
        data = load_json_file(json_path)
        completed_count = completed_waypoint_count(data, json_path)
        if completed_count == 0:
            invalid_files.append(json_path)
            continue

        if completed_count == EXPECTED_COMPLETED_WAYPOINTS:
            widths = envelope_widths_from_completed_file(data, json_path)
            successful_files.append(json_path)
            widths_m.extend(widths)
        else:
            failed_files.append(json_path)

        if len(successful_files) + len(failed_files) >= max_valid_json_files:
            break

    if not successful_files and not failed_files:
        raise RuntimeError("no valid JSON files had completed_waypoint_count != 0")

    return EnvelopeWidthSamples(
        successful_files=tuple(successful_files),
        failed_files=tuple(failed_files),
        invalid_files=tuple(invalid_files),
        unprocessed_file_count=len(json_paths) - processed_file_count,
        widths_m=tuple(widths_m),
    )


def deterministic_jittered_x_values(count: int) -> tuple[float, ...]:
    if count <= 0:
        return tuple()
    if count == 1:
        return (SCATTER_CENTER_X,)

    modulus = 2147483647
    multiplier = 48271
    value = 1
    x_values: list[float] = []
    for _ in range(count):
        value = (multiplier * value) % modulus
        normalized = value / modulus
        jitter = (normalized - 0.5) * 2.0 * SCATTER_JITTER_HALF_WIDTH
        x_values.append(SCATTER_CENTER_X + jitter)
    return tuple(x_values)


def plot_envelope_width_scatter(samples: EnvelopeWidthSamples, plt):
    configure_plot_style(plt)

    fig, ax = plt.subplots(
        figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH),
        constrained_layout=True,
    )
    ax.scatter(
        deterministic_jittered_x_values(len(samples.widths_m)),
        samples.widths_m,
        s=SCATTER_MARKER_SIZE_PT2,
        color=SCATTER_COLOR,
        edgecolors=SCATTER_EDGE_COLOR,
        linewidths=SCATTER_EDGE_LINEWIDTH,
        alpha=SCATTER_ALPHA,
        zorder=3,
    )

    ax.set_xlim(
        SCATTER_CENTER_X - SCATTER_JITTER_HALF_WIDTH * 1.25,
        SCATTER_CENTER_X + SCATTER_JITTER_HALF_WIDTH * 1.25,
    )
    ax.set_xticks([])
    ax.tick_params(axis="x", which="both", bottom=False, labelbottom=False)
    ax.spines["bottom"].set_visible(False)
    ax.set_ylabel("Envelope width [m]")
    ax.grid(True, which="major", axis="y", color="0.82", linewidth=0.45, alpha=0.7)
    ax.set_axisbelow(True)
    ax.margins(y=0.05)
    return fig


def resolve_output_pdf_path(output_dir: Path, output_filename: str) -> Path:
    directory = Path(output_dir).expanduser().resolve()
    return directory / output_filename


def print_summary(samples: EnvelopeWidthSamples) -> None:
    widths = samples.widths_m
    successful_count = len(samples.successful_files)
    failed_count = len(samples.failed_files)
    valid_count = successful_count + failed_count
    success_rate = successful_count / valid_count
    failed_case_ids = tuple(case_id_from_json_path(path) for path in samples.failed_files)

    print("Valid JSON file limit: {}".format(MAX_VALID_JSON_FILES))
    print("Valid JSON files: {}".format(valid_count))
    print("Successful attempts: {}".format(successful_count))
    print("Failed attempts: {}".format(failed_count))
    print("Invalid JSON files: {}".format(len(samples.invalid_files)))
    print("Unprocessed JSON files after limit: {}".format(samples.unprocessed_file_count))
    print("Success rate: {:.6f} ({:.2f}%)".format(success_rate, success_rate * 100.0))
    print(
        "Failed case IDs: {}".format(
            ", ".join(failed_case_ids) if failed_case_ids else "none"
        )
    )
    print("Envelope width data points: {}".format(len(widths)))
    if widths:
        print(
            "Envelope width [m]: min={:.6f}, mean={:.6f}, max={:.6f}".format(
                min(widths),
                statistics.fmean(widths),
                max(widths),
            )
        )
    else:
        print("Envelope width [m]: no successful attempts")


def main() -> int:
    try:
        samples = load_envelope_width_samples(DATA_ROOT, MAX_VALID_JSON_FILES)
        print_summary(samples)

        plt = import_pyplot(not SHOW_PLOT)
        fig = plot_envelope_width_scatter(samples, plt)

        if SAVE_FIGURE:
            output_path = resolve_output_pdf_path(OUTPUT_DIR, OUTPUT_FILENAME)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(output_path, dpi=SAVE_DPI)
            print("Saved plot: {}".format(output_path))

        if SHOW_PLOT:
            plt.show()

        plt.close(fig)
        return 0
    except RuntimeError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
