#!/usr/bin/env python3
"""Timing / RSS analyzer for LIO logs."""
import argparse
import csv
import math
import re
from collections import defaultdict
from statistics import mean

STAGE_PATTERN = re.compile(r"Timing\] (.+?)\s+(start|end)\s+(\d+\.\d+)")
FRAME_PATTERN = re.compile(r"Frame: (\d+)")
TIMESTAMP_PATTERN = re.compile(r"\[(\d+\.\d+)\]")


def analyze_log(path: str):
    stats = defaultdict(lambda: {"durations": [], "last_start": None})
    frame_intervals = []
    last_frame_ts = None

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = STAGE_PATTERN.search(line)
            if m:
                stage, marker, ts_str = m.groups()
                ts = float(ts_str)
                entry = stats[stage]
                if marker == "start":
                    entry["last_start"] = ts
                else:
                    start = entry.get("last_start")
                    if start is not None:
                        entry["durations"].append(ts - start)
                        entry["last_start"] = None
                continue

            m = FRAME_PATTERN.search(line)
            if m:
                ts_match = TIMESTAMP_PATTERN.search(line)
                if ts_match:
                    ts = float(ts_match.group(1))
                    if last_frame_ts is not None:
                        frame_intervals.append(ts - last_frame_ts)
                    last_frame_ts = ts

    summary = []
    for stage, entry in stats.items():
        durations = entry["durations"]
        if durations:
            summary.append(
                {
                    "stage": stage,
                    "count": len(durations),
                    "avg": mean(durations),
                    "max": max(durations),
                }
            )
    summary.sort(key=lambda x: x["avg"], reverse=True)
    return summary, frame_intervals


def analyze_rss(path: str):
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [(row["timestamp"], int(row["rss_mb"])) for row in reader]
    if not rows:
        return None
    growth_segments = []
    window = 30  # approximate 30 samples (~30s with 1 Hz sampling)
    for i in range(0, len(rows), window):
        chunk = rows[i : i + window]
        if len(chunk) >= 2:
            growth_segments.append(
                {
                    "start": chunk[0][0],
                    "delta": chunk[-1][1] - chunk[0][1],
                }
            )
    max_delta = 0
    max_ts = None
    for (prev_ts, prev_val), (curr_ts, curr_val) in zip(rows, rows[1:]):
        delta = curr_val - prev_val
        if delta > max_delta:
            max_delta = delta
            max_ts = curr_ts
    report = {
        "samples": len(rows),
        "first": rows[0][1],
        "last": rows[-1][1],
        "delta": rows[-1][1] - rows[0][1],
        "segments": growth_segments,
        "max_delta": max_delta,
        "max_delta_ts": max_ts,
    }
    return report


def format_ms(value: float) -> str:
    return f"{value * 1000:8.3f} ms"


def main():
    parser = argparse.ArgumentParser(description="Analyze LIO timing log and RSS CSV")
    parser.add_argument("--log", required=True, help="Path to timing log (with [Timing] entries)")
    parser.add_argument("--rss", help="Path to RSS CSV (from capture_pose_bt.sh)")
    args = parser.parse_args()

    summary, frame_intervals = analyze_log(args.log)
    print("Stage Summary (avg / max in ms):")
    for entry in summary:
        print(
            f"{entry['stage']:30s} count={entry['count']:6d} "
            f"avg={format_ms(entry['avg'])} max={format_ms(entry['max'])}"
        )

    if frame_intervals:
        print("\nFrame interval stats:")
        print(
            f"frames={len(frame_intervals)} "
            f"avg={format_ms(mean(frame_intervals))} "
            f"max={format_ms(max(frame_intervals))} "
            f"min={format_ms(min(frame_intervals))}"
        )

    if args.rss:
        report = analyze_rss(args.rss)
        if report:
            print("\nRSS stats:")
            print(
                f"samples={report['samples']} first={report['first']} MB last={report['last']} MB "
                f"delta={report['delta']} MB"
            )
            print("Growth per ~30 samples (~30s):")
            for seg in report["segments"]:
                print(f"  {seg['start']}: {seg['delta']:4d} MB")
            if report["max_delta_ts"]:
                print(
                    f"Max second-to-second increase: {report['max_delta']} MB at "
                    f"{report['max_delta_ts']}"
                )
        else:
            print("\nRSS stats: file empty or invalid.")


if __name__ == "__main__":
    main()
