#!/usr/bin/env python3

"""Clean PlotJuggler-exported rosbag CSV files."""

from __future__ import annotations

import argparse
import csv
import os
import tempfile
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterable, Sequence


ROSOUT_PREFIXES = ("rosout/", "rosout_agg/")


@dataclass(frozen=True)
class ScanResult:
    input_rows: int
    dropped_empty_rows: int
    dropped_leading_rosout_rows: int
    zero_time: Decimal | None
    zero_time_text: str | None
    header_width: int
    non_rosout_indices: tuple[int, ...]


@dataclass(frozen=True)
class FileStats:
    file_name: str
    input_rows: int
    output_rows: int
    dropped_empty_rows: int
    dropped_leading_rosout_rows: int
    zero_time_text: str | None
    skipped: bool
    skip_reason: str | None = None


@dataclass
class SummaryStats:
    total_files: int = 0
    written_files: int = 0
    skipped_files: int = 0
    failed_files: int = 0
    total_input_rows: int = 0
    total_output_rows: int = 0
    total_dropped_empty_rows: int = 0
    total_dropped_leading_rosout_rows: int = 0


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    package_dir = script_dir.parent
    parser = argparse.ArgumentParser(
        description="Clean PlotJuggler-exported rosbag CSV files."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=package_dir / "data" / "rosbag_csv_raw",
        help="Directory containing raw PlotJuggler CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=package_dir / "data" / "rosbag_csv",
        help="Directory to write cleaned CSV files to.",
    )
    return parser.parse_args(argv)


def is_empty_cell(cell: str) -> bool:
    return cell.strip() == ""


def is_rosout_column(column_name: str) -> bool:
    normalized = column_name.lstrip("/")
    return normalized.startswith(ROSOUT_PREFIXES)


def parse_decimal(value: str, input_path: Path) -> Decimal:
    stripped = value.strip()
    if not stripped:
        raise ValueError(f"{input_path.name}: empty __time cell encountered")
    try:
        return Decimal(stripped)
    except InvalidOperation as exc:
        raise ValueError(
            f"{input_path.name}: invalid __time value '{value}'"
        ) from exc


def decimal_places(value: str) -> int:
    stripped = value.strip()
    if not stripped:
        return 0
    mantissa = stripped.lower().split("e", 1)[0]
    if "." not in mantissa:
        return 0
    return len(mantissa.split(".", 1)[1])


def format_shifted_time(original_time: str, zero_time: Decimal, input_path: Path) -> str:
    places = decimal_places(original_time)
    shifted = parse_decimal(original_time, input_path) - zero_time
    quantizer = Decimal(1).scaleb(-places) if places > 0 else Decimal(1)
    shifted = shifted.quantize(quantizer)
    if shifted == 0:
        shifted = abs(shifted)
    if places > 0:
        return format(shifted, f".{places}f")
    return format(shifted, "f")


def normalize_row(row: list[str], header_width: int, input_path: Path) -> list[str]:
    if len(row) > header_width:
        raise ValueError(
            f"{input_path.name}: row has {len(row)} columns, expected {header_width}"
        )
    if len(row) < header_width:
        row = row + [""] * (header_width - len(row))
    return row


def open_csv_reader(input_path: Path):
    return input_path.open("r", newline="", encoding="utf-8")


def iter_non_recursive_csv_files(input_dir: Path) -> Iterable[Path]:
    return sorted(
        (path for path in input_dir.iterdir() if path.is_file() and path.suffix.lower() == ".csv"),
        key=lambda path: path.name,
    )


def scan_csv_file(input_path: Path) -> ScanResult:
    with open_csv_reader(input_path) as input_file:
        reader = csv.reader(input_file)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"{input_path.name}: empty CSV file") from exc

        if not header:
            raise ValueError(f"{input_path.name}: missing CSV header")
        if header[0] != "__time":
            raise ValueError(
                f"{input_path.name}: first column must be '__time', got '{header[0]}'"
            )

        non_rosout_indices = tuple(
            index
            for index, column_name in enumerate(header)
            if index != 0 and not is_rosout_column(column_name)
        )

        input_rows = 0
        dropped_empty_rows = 0
        dropped_leading_rosout_rows = 0
        zero_time = None
        zero_time_text = None

        for row in reader:
            row = normalize_row(row, len(header), input_path)
            input_rows += 1

            has_any_payload = any(not is_empty_cell(cell) for cell in row[1:])
            if not has_any_payload:
                dropped_empty_rows += 1
                continue

            has_non_rosout_payload = any(
                not is_empty_cell(row[index]) for index in non_rosout_indices
            )
            if zero_time is None:
                if has_non_rosout_payload:
                    zero_time_text = row[0].strip()
                    zero_time = parse_decimal(row[0], input_path)
                else:
                    dropped_leading_rosout_rows += 1

        return ScanResult(
            input_rows=input_rows,
            dropped_empty_rows=dropped_empty_rows,
            dropped_leading_rosout_rows=dropped_leading_rosout_rows,
            zero_time=zero_time,
            zero_time_text=zero_time_text,
            header_width=len(header),
            non_rosout_indices=non_rosout_indices,
        )


def write_cleaned_csv(input_path: Path, output_path: Path, scan_result: ScanResult) -> int:
    if scan_result.zero_time is None:
        raise ValueError(f"{input_path.name}: zero_time is required to write output")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    output_rows = 0
    started = False
    temp_file = None
    temp_path = None

    try:
        with open_csv_reader(input_path) as input_file:
            reader = csv.reader(input_file)
            header = next(reader)

            temp_file = tempfile.NamedTemporaryFile(
                "w",
                newline="",
                encoding="utf-8",
                delete=False,
                dir=output_path.parent,
                prefix=f".{output_path.stem}_",
                suffix=".tmp",
            )
            temp_path = Path(temp_file.name)

            with temp_file:
                writer = csv.writer(temp_file, lineterminator="\n")
                writer.writerow(header)

                for row in reader:
                    row = normalize_row(row, scan_result.header_width, input_path)
                    has_any_payload = any(not is_empty_cell(cell) for cell in row[1:])
                    if not has_any_payload:
                        continue

                    has_non_rosout_payload = any(
                        not is_empty_cell(row[index])
                        for index in scan_result.non_rosout_indices
                    )

                    if not started:
                        if not has_non_rosout_payload:
                            continue
                        started = True

                    row[0] = format_shifted_time(row[0], scan_result.zero_time, input_path)
                    writer.writerow(row)
                    output_rows += 1

        os.replace(temp_path, output_path)
        return output_rows
    except Exception:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()
        raise


def process_csv_file(input_path: Path, output_path: Path) -> FileStats:
    scan_result = scan_csv_file(input_path)
    if scan_result.zero_time is None:
        if output_path.exists():
            output_path.unlink()
        return FileStats(
            file_name=input_path.name,
            input_rows=scan_result.input_rows,
            output_rows=0,
            dropped_empty_rows=scan_result.dropped_empty_rows,
            dropped_leading_rosout_rows=scan_result.dropped_leading_rosout_rows,
            zero_time_text=None,
            skipped=True,
            skip_reason="no non-rosout topic data found",
        )

    output_rows = write_cleaned_csv(input_path, output_path, scan_result)
    return FileStats(
        file_name=input_path.name,
        input_rows=scan_result.input_rows,
        output_rows=output_rows,
        dropped_empty_rows=scan_result.dropped_empty_rows,
        dropped_leading_rosout_rows=scan_result.dropped_leading_rosout_rows,
        zero_time_text=scan_result.zero_time_text,
        skipped=False,
    )


def log_file_stats(stats: FileStats) -> None:
    if stats.skipped:
        print(
            f"[SKIP] {stats.file_name}: input_rows={stats.input_rows}, "
            f"dropped_empty_rows={stats.dropped_empty_rows}, "
            f"dropped_leading_rosout_rows={stats.dropped_leading_rosout_rows}, "
            f"reason={stats.skip_reason}"
        )
        return

    print(
        f"[OK] {stats.file_name}: input_rows={stats.input_rows}, "
        f"output_rows={stats.output_rows}, "
        f"dropped_empty_rows={stats.dropped_empty_rows}, "
        f"dropped_leading_rosout_rows={stats.dropped_leading_rosout_rows}, "
        f"zero_time={stats.zero_time_text}"
    )


def process_directory(input_dir: Path, output_dir: Path) -> int:
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory does not exist: {input_dir}")
    if not input_dir.is_dir():
        raise NotADirectoryError(f"Input path is not a directory: {input_dir}")

    output_dir.mkdir(parents=True, exist_ok=True)

    csv_files = list(iter_non_recursive_csv_files(input_dir))
    if not csv_files:
        print(f"No CSV files found in {input_dir}")
        return 0

    summary = SummaryStats(total_files=len(csv_files))
    print(f"Processing {len(csv_files)} CSV file(s) from {input_dir} -> {output_dir}")

    for input_path in csv_files:
        output_path = output_dir / input_path.name
        try:
            stats = process_csv_file(input_path, output_path)
        except Exception as exc:
            summary.failed_files += 1
            print(f"[ERROR] {input_path.name}: {exc}")
            continue

        summary.total_input_rows += stats.input_rows
        summary.total_output_rows += stats.output_rows
        summary.total_dropped_empty_rows += stats.dropped_empty_rows
        summary.total_dropped_leading_rosout_rows += stats.dropped_leading_rosout_rows

        if stats.skipped:
            summary.skipped_files += 1
        else:
            summary.written_files += 1

        log_file_stats(stats)

    print(
        "Summary: "
        f"files={summary.total_files}, "
        f"written={summary.written_files}, "
        f"skipped={summary.skipped_files}, "
        f"failed={summary.failed_files}, "
        f"input_rows={summary.total_input_rows}, "
        f"output_rows={summary.total_output_rows}, "
        f"dropped_empty_rows={summary.total_dropped_empty_rows}, "
        f"dropped_leading_rosout_rows={summary.total_dropped_leading_rosout_rows}"
    )

    return 0 if summary.failed_files == 0 else 1


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    return process_directory(args.input_dir.resolve(), args.output_dir.resolve())


if __name__ == "__main__":
    raise SystemExit(main())
