#!/usr/bin/env python3
"""Timing / RSS analyzer for LIO logs."""
import argparse
import csv
import re
from collections import defaultdict
from statistics import mean

STAGE_PATTERN = re.compile(r"Timing\] (.+?)\s+(start|end)\s+(\d+\.\d+)")
FRAME_PATTERN = re.compile(r"Frame: (\d+)")
TIMESTAMP_PATTERN = re.compile(r"\[(\d+\.\d+)\]")

# Livox 点云频率 (Hz)
LIDAR_FREQUENCY = 10.0


def analyze_log(path: str):
    stats = defaultdict(lambda: {"durations": [], "last_start": None})
    frame_intervals = []
    frame_timestamps = []
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
                    frame_timestamps.append(ts)
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
                    "total": sum(durations),
                }
            )
    summary.sort(key=lambda x: x["avg"], reverse=True)
    return summary, frame_intervals, frame_timestamps


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
        "peak": max(r[1] for r in rows),
        "delta": rows[-1][1] - rows[0][1],
        "segments": growth_segments,
        "max_delta": max_delta,
        "max_delta_ts": max_ts,
    }
    return report


def format_ms(value: float) -> str:
    return f"{value * 1000:8.3f} ms"


def format_time(seconds: float) -> str:
    """格式化时间为 mm:ss.sss 或 hh:mm:ss.sss"""
    if seconds < 3600:
        mins = int(seconds // 60)
        secs = seconds % 60
        return f"{mins:02d}:{secs:06.3f}"
    else:
        hours = int(seconds // 3600)
        mins = int((seconds % 3600) // 60)
        secs = seconds % 60
        return f"{hours:02d}:{mins:02d}:{secs:06.3f}"


def main():
    parser = argparse.ArgumentParser(description="Analyze LIO timing log and RSS CSV")
    parser.add_argument("--log", required=True, help="Path to timing log (with [Timing] entries)")
    parser.add_argument("--rss", help="Path to RSS CSV (from capture_pose_bt.sh)")
    args = parser.parse_args()

    summary, frame_intervals, frame_timestamps = analyze_log(args.log)

    print("=" * 70)
    print(" LIO 性能分析报告")
    print("=" * 70)

    print("\n【各阶段耗时统计】(avg / max in ms)")
    print("-" * 70)
    for entry in summary:
        print(
            f"{entry['stage']:30s} count={entry['count']:6d} "
            f"avg={format_ms(entry['avg'])} max={format_ms(entry['max'])}"
        )

    if frame_intervals:
        print("\n【帧间隔统计】")
        print("-" * 70)
        print(
            f"总帧数: {len(frame_intervals) + 1}\n"
            f"帧间隔: avg={format_ms(mean(frame_intervals))} "
            f"max={format_ms(max(frame_intervals))} "
            f"min={format_ms(min(frame_intervals))}"
        )

    # 实时性分析
    if frame_timestamps and len(frame_timestamps) >= 2:
        print("\n【实时性分析】")
        print("-" * 70)

        total_frames = len(frame_timestamps)

        # 算法处理总时长 = 最后一帧时间戳 - 第一帧时间戳
        processing_duration = frame_timestamps[-1] - frame_timestamps[0]

        # Rosbag 理论时长 = 帧数 / 10Hz (Livox 点云频率)
        # 注意：帧间隔数 = 帧数 - 1，所以用 (total_frames - 1) / 10
        rosbag_duration = (total_frames - 1) / LIDAR_FREQUENCY

        # 实时性比率
        realtime_ratio = processing_duration / rosbag_duration if rosbag_duration > 0 else 0

        # 判断是否实时
        is_realtime = realtime_ratio <= 1.0
        realtime_status = "✓ 实时" if is_realtime else "✗ 非实时"

        # 计算超时/富余
        time_diff = processing_duration - rosbag_duration
        time_diff_str = f"+{time_diff:.3f}s (落后)" if time_diff > 0 else f"{time_diff:.3f}s (富余)"

        print(f"总帧数:              {total_frames} 帧")
        print(f"Rosbag 理论时长:     {format_time(rosbag_duration)} ({rosbag_duration:.3f}s) @ {LIDAR_FREQUENCY:.0f}Hz")
        print(f"算法处理总时长:      {format_time(processing_duration)} ({processing_duration:.3f}s)")
        print(f"时间差:              {time_diff_str}")
        print(f"实时性比率:          {realtime_ratio:.4f}x  {realtime_status}")

        # 计算需要达到实时的目标帧处理时间
        if not is_realtime:
            current_avg_ms = mean(frame_intervals) * 1000
            target_avg_ms = 1000.0 / LIDAR_FREQUENCY  # 100ms for 10Hz
            reduction_needed = current_avg_ms - target_avg_ms
            print(f"\n需优化: 平均帧处理时间需从 {current_avg_ms:.1f}ms 降至 <{target_avg_ms:.0f}ms (减少 {reduction_needed:.1f}ms)")

    if args.rss:
        report = analyze_rss(args.rss)
        if report:
            print("\n【内存统计 (RSS)】")
            print("-" * 70)
            print(
                f"采样数: {report['samples']}\n"
                f"初始:   {report['first']} MB\n"
                f"最终:   {report['last']} MB\n"
                f"峰值:   {report['peak']} MB\n"
                f"增长:   {report['delta']} MB"
            )
            if report["max_delta_ts"]:
                print(
                    f"最大瞬时增长: {report['max_delta']} MB @ {report['max_delta_ts']}"
                )
        else:
            print("\n【内存统计】: 文件为空或格式错误")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
