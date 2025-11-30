#!/usr/bin/env python3
"""Timing / RSS analyzer for LIO logs.

支持的时间戳模块:
  - Estimator::Estimate
  - Residual build
  - Ceres solve
  - MapManager update
  - MapManager snapshot
  - Marginalization
  - RemoveDistortion
  - IMU_GyroIntegration
  - IMU_PreIntegration
  - FeatureExtract (ScanRegistration 节点)

FIFO 丢帧统计:
  - LiDAR queue overflow
  - IMU queue overflow
  - LiDAR msg too old
"""
import argparse
import csv
import re
from collections import defaultdict
from statistics import mean

STAGE_PATTERN = re.compile(r"Timing\] (.+?)\s+(start|end)\s+(\d+\.\d+)")
FRAME_PATTERN = re.compile(r"Frame: (\d+)")
TIMESTAMP_PATTERN = re.compile(r"\[(\d+\.\d+)\]")

# FIFO 丢帧统计模式
FIFO_LIDAR_DROP_PATTERN = re.compile(r"\[FIFO\] LiDAR queue overflow.*total dropped: (\d+)")
FIFO_LIDAR_LAG_PATTERN = re.compile(r"\[FIFO\] LiDAR msg too old.*total: (\d+)")
FIFO_IMU_DROP_PATTERN = re.compile(r"\[FIFO\] IMU queue overflow.*total dropped: (\d+)")

# Livox 点云频率 (Hz)
LIDAR_FREQUENCY = 10.0

# 模块分组 (用于分类输出)
MODULE_GROUPS = {
    "特征提取": ["FeatureExtract"],
    "位姿估计": ["Estimator::Estimate", "Residual build", "Ceres solve"],
    "地图管理": ["MapManager update", "MapManager snapshot"],
    "边缘化": ["Marginalization"],
    "畸变校正": ["RemoveDistortion"],
    "IMU 积分": ["IMU_GyroIntegration", "IMU_PreIntegration"],
}


def analyze_log(path: str):
    stats = defaultdict(lambda: {"durations": [], "last_start": None})
    frame_intervals = []
    frame_timestamps = []
    last_frame_ts = None

    # FIFO 丢帧统计
    fifo_stats = {
        "lidar_queue_drop": 0,
        "lidar_lag_drop": 0,
        "imu_drop": 0,
    }

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
                continue

            # FIFO 丢帧统计
            m = FIFO_LIDAR_DROP_PATTERN.search(line)
            if m:
                fifo_stats["lidar_queue_drop"] = max(fifo_stats["lidar_queue_drop"], int(m.group(1)))
                continue

            m = FIFO_LIDAR_LAG_PATTERN.search(line)
            if m:
                fifo_stats["lidar_lag_drop"] = max(fifo_stats["lidar_lag_drop"], int(m.group(1)))
                continue

            m = FIFO_IMU_DROP_PATTERN.search(line)
            if m:
                fifo_stats["imu_drop"] = max(fifo_stats["imu_drop"], int(m.group(1)))
                continue

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
                    "min": min(durations),
                    "total": sum(durations),
                }
            )
    summary.sort(key=lambda x: x["avg"], reverse=True)
    return summary, frame_intervals, frame_timestamps, fifo_stats


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


def get_module_group(stage: str) -> str:
    """获取模块所属分组"""
    for group, modules in MODULE_GROUPS.items():
        if stage in modules:
            return group
    return "其他"


def main():
    parser = argparse.ArgumentParser(description="Analyze LIO timing log and RSS CSV")
    parser.add_argument("--log", required=True, help="Path to timing log (with [Timing] entries)")
    parser.add_argument("--rss", help="Path to RSS CSV (from capture_pose_bt.sh)")
    parser.add_argument("--group", action="store_true", help="Group output by module category")
    args = parser.parse_args()

    summary, frame_intervals, frame_timestamps, fifo_stats = analyze_log(args.log)

    print("=" * 70)
    print(" LIO 性能分析报告")
    print("=" * 70)

    print("\n【各阶段耗时统计】(avg / max / min in ms)")
    print("-" * 70)

    if args.group:
        # 按分组输出
        grouped = defaultdict(list)
        for entry in summary:
            group = get_module_group(entry["stage"])
            grouped[group].append(entry)

        # 按分组顺序输出
        group_order = list(MODULE_GROUPS.keys()) + ["其他"]
        for group in group_order:
            if group in grouped and grouped[group]:
                print(f"\n  [{group}]")
                for entry in grouped[group]:
                    print(
                        f"    {entry['stage']:28s} count={entry['count']:6d} "
                        f"avg={format_ms(entry['avg'])} max={format_ms(entry['max'])} min={format_ms(entry['min'])}"
                    )
    else:
        # 原有平铺输出
        for entry in summary:
            print(
                f"{entry['stage']:30s} count={entry['count']:6d} "
                f"avg={format_ms(entry['avg'])} max={format_ms(entry['max'])} min={format_ms(entry['min'])}"
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

    # FIFO 丢帧统计
    total_fifo_drops = fifo_stats["lidar_queue_drop"] + fifo_stats["lidar_lag_drop"] + fifo_stats["imu_drop"]
    if total_fifo_drops > 0:
        print("\n【FIFO 丢帧统计】")
        print("-" * 70)
        print(f"LiDAR 队列溢出丢帧:   {fifo_stats['lidar_queue_drop']:6d} 帧")
        print(f"LiDAR 延迟过大丢帧:   {fifo_stats['lidar_lag_drop']:6d} 帧")
        print(f"IMU 队列溢出丢帧:     {fifo_stats['imu_drop']:6d} 条")
        print(f"总计丢弃:             {total_fifo_drops:6d}")

        # 如果有帧统计，计算丢帧率
        if frame_timestamps:
            total_frames = len(frame_timestamps)
            lidar_drops = fifo_stats["lidar_queue_drop"] + fifo_stats["lidar_lag_drop"]
            drop_rate = lidar_drops / (total_frames + lidar_drops) * 100 if (total_frames + lidar_drops) > 0 else 0
            print(f"LiDAR 丢帧率:         {drop_rate:6.2f}%")

            if drop_rate > 10:
                print("\n警告: 丢帧率较高，建议:")
                print("  - 降低 max_corner_residuals / max_surf_residuals")
                print("  - 增大 map_skip_frame")
                print("  - 减小 local_box_* 范围")

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
