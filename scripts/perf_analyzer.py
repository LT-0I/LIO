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

# 残差分析正则表达式
CORNER_ADAPTIVE_PATTERN = re.compile(
    r"Estimator corner adaptive iter (\d+): mode=(\w+) avg_global_kd=([\d.]+) limit=(\d+) eigen_ratio=([\d.]+)"
)
CORNER_BUILD_STATS_PATTERN = re.compile(
    r"Estimator corner build stats iter (\d+): points=(\d+) region_skip=(\d+) "
    r"global_kd=(\d+) global_pass=(\d+) global_fail=(\d+) "
    r"local_kd=(\d+) local_pass=(\d+) local_fail=(\d+)"
)
RESIDUAL_CANDIDATES_PATTERN = re.compile(
    r"Estimator residual candidates iter (\d+): corner=(\d+) \(global=(\d+) local=(\d+)\) surf=(\d+) non=(\d+)"
)
RESIDUAL_KEPT_PATTERN = re.compile(
    r"Estimator residual kept iter (\d+): corner=(\d+) \(global=(\d+) local=(\d+)\) surf=(\d+) non=(\d+)"
)

# 漂移诊断正则表达式
POSE_DELTA_PATTERN = re.compile(
    r"Estimator pose_delta: deltaR=([\d.]+) deg deltaT=([\d.]+) m speed=([\d.]+) m/s iters=(\d+)"
)
CORNER_ADAPTIVE_FULL_PATTERN = re.compile(
    r"Estimator corner adaptive iter (\d+): mode=(\w+) avg_global_kd=([\d.]+) limit=(\d+) "
    r"eigen_ratio=([\d.]+) thres_dist=([\d.]+) plan_w=([\d.]+)"
)

# Livox 点云频率 (Hz)
LIDAR_FREQUENCY = 10.0


def analyze_log(path: str):
    stats = defaultdict(lambda: {"durations": [], "last_start": None})
    frame_intervals = []
    frame_timestamps = []
    last_frame_ts = None

    # 残差分析数据结构
    residual_data = {
        "adaptive_modes": [],  # [(frame, iter, mode, avg_global_kd, limit, eigen_ratio, thres_dist, plan_w), ...]
        "build_stats": [],     # [(frame, points, global_kd, global_pass, global_fail, local_kd, local_pass, local_fail), ...]
        "candidates": [],      # [(frame, iter, corner, corner_global, corner_local, surf, non), ...]
        "kept": [],            # [(frame, iter, corner, corner_global, corner_local, surf, non), ...]
        "pose_delta": [],      # [(frame, deltaR, deltaT, speed, iters), ...]
    }
    current_frame = -1

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            # 时序分析
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

            # 帧号提取
            m = FRAME_PATTERN.search(line)
            if m:
                current_frame = int(m.group(1))
                ts_match = TIMESTAMP_PATTERN.search(line)
                if ts_match:
                    ts = float(ts_match.group(1))
                    frame_timestamps.append(ts)
                    if last_frame_ts is not None:
                        frame_intervals.append(ts - last_frame_ts)
                    last_frame_ts = ts
                continue

            # 角点自适应模式 (完整版带 thres_dist 和 plan_w)
            m = CORNER_ADAPTIVE_FULL_PATTERN.search(line)
            if m:
                iter_num, mode, avg_global_kd, limit, eigen_ratio, thres_dist, plan_w = m.groups()
                residual_data["adaptive_modes"].append({
                    "frame": current_frame,
                    "iter": int(iter_num),
                    "mode": mode,
                    "avg_global_kd": float(avg_global_kd),
                    "limit": int(limit),
                    "eigen_ratio": float(eigen_ratio),
                    "thres_dist": float(thres_dist),
                    "plan_w": float(plan_w),
                })
                continue
            # 角点自适应模式 (旧版兼容)
            m = CORNER_ADAPTIVE_PATTERN.search(line)
            if m:
                iter_num, mode, avg_global_kd, limit, eigen_ratio = m.groups()
                residual_data["adaptive_modes"].append({
                    "frame": current_frame,
                    "iter": int(iter_num),
                    "mode": mode,
                    "avg_global_kd": float(avg_global_kd),
                    "limit": int(limit),
                    "eigen_ratio": float(eigen_ratio),
                    "thres_dist": 1.0,  # 默认值
                    "plan_w": 0.0003,   # 默认值
                })
                continue

            # 角点构建统计 (仅 iter 0)
            m = CORNER_BUILD_STATS_PATTERN.search(line)
            if m:
                iter_num, points, region_skip, global_kd, global_pass, global_fail, local_kd, local_pass, local_fail = m.groups()
                if int(iter_num) == 0:
                    residual_data["build_stats"].append({
                        "frame": current_frame,
                        "points": int(points),
                        "global_kd": int(global_kd),
                        "global_pass": int(global_pass),
                        "global_fail": int(global_fail),
                        "local_kd": int(local_kd),
                        "local_pass": int(local_pass),
                        "local_fail": int(local_fail),
                    })
                continue

            # 残差候选数量
            m = RESIDUAL_CANDIDATES_PATTERN.search(line)
            if m:
                iter_num, corner, corner_global, corner_local, surf, non = m.groups()
                residual_data["candidates"].append({
                    "frame": current_frame,
                    "iter": int(iter_num),
                    "corner": int(corner),
                    "corner_global": int(corner_global),
                    "corner_local": int(corner_local),
                    "surf": int(surf),
                    "non": int(non),
                })
                continue

            # 残差保留数量
            m = RESIDUAL_KEPT_PATTERN.search(line)
            if m:
                iter_num, corner, corner_global, corner_local, surf, non = m.groups()
                residual_data["kept"].append({
                    "frame": current_frame,
                    "iter": int(iter_num),
                    "corner": int(corner),
                    "corner_global": int(corner_global),
                    "corner_local": int(corner_local),
                    "surf": int(surf),
                    "non": int(non),
                })
                continue

            # 位姿变化 (漂移诊断)
            m = POSE_DELTA_PATTERN.search(line)
            if m:
                deltaR, deltaT, speed, iters = m.groups()
                residual_data["pose_delta"].append({
                    "frame": current_frame,
                    "deltaR": float(deltaR),
                    "deltaT": float(deltaT),
                    "speed": float(speed),
                    "iters": int(iters),
                })
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
                    "total": sum(durations),
                }
            )
    summary.sort(key=lambda x: x["avg"], reverse=True)
    return summary, frame_intervals, frame_timestamps, residual_data


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


def analyze_residuals(residual_data: dict):
    """分析残差统计信息并生成报告"""
    report = {}

    # 1. 残差数量统计 (仅统计 iter 0 的数据)
    candidates_iter0 = [c for c in residual_data["candidates"] if c["iter"] == 0]
    kept_iter0 = [k for k in residual_data["kept"] if k["iter"] == 0]

    if candidates_iter0:
        report["candidates"] = {
            "corner": {"avg": mean([c["corner"] for c in candidates_iter0]),
                      "max": max([c["corner"] for c in candidates_iter0]),
                      "min": min([c["corner"] for c in candidates_iter0])},
            "surf": {"avg": mean([c["surf"] for c in candidates_iter0]),
                    "max": max([c["surf"] for c in candidates_iter0]),
                    "min": min([c["surf"] for c in candidates_iter0])},
            "non": {"avg": mean([c["non"] for c in candidates_iter0]),
                   "max": max([c["non"] for c in candidates_iter0]),
                   "min": min([c["non"] for c in candidates_iter0])},
        }

    if kept_iter0:
        report["kept"] = {
            "corner": {"avg": mean([k["corner"] for k in kept_iter0]),
                      "max": max([k["corner"] for k in kept_iter0]),
                      "min": min([k["corner"] for k in kept_iter0])},
            "surf": {"avg": mean([k["surf"] for k in kept_iter0]),
                    "max": max([k["surf"] for k in kept_iter0]),
                    "min": min([k["surf"] for k in kept_iter0])},
            "non": {"avg": mean([k["non"] for k in kept_iter0]),
                   "max": max([k["non"] for k in kept_iter0]),
                   "min": min([k["non"] for k in kept_iter0])},
        }

    # 2. 角点自适应模式统计 (仅 iter 0)
    modes_iter0 = [m for m in residual_data["adaptive_modes"] if m["iter"] == 0]
    if modes_iter0:
        mode_counts = defaultdict(lambda: {"count": 0, "avg_global_kd_sum": 0.0})
        for m in modes_iter0:
            mode_counts[m["mode"]]["count"] += 1
            mode_counts[m["mode"]]["avg_global_kd_sum"] += m["avg_global_kd"]

        report["modes"] = {}
        total_frames = len(modes_iter0)
        for mode, data in mode_counts.items():
            report["modes"][mode] = {
                "count": data["count"],
                "percent": data["count"] / total_frames * 100,
                "avg_global_kd": data["avg_global_kd_sum"] / data["count"] if data["count"] > 0 else 0,
            }

    # 3. 低特征帧识别 (corner_kept < 100 且 iter == 0)
    low_feature_frames = []
    for k in kept_iter0:
        if k["corner"] < 100:
            low_feature_frames.append(k)
    report["low_feature_frames"] = low_feature_frames

    # 4. 仅局部角点帧 (global=0 且 iter == 0)
    local_only_frames = []
    for k in kept_iter0:
        if k["corner_global"] == 0 and k["corner"] > 0:
            local_only_frames.append(k)
    report["local_only_frames"] = local_only_frames

    # 5. 全局/局部 KD 匹配统计
    build_stats = residual_data["build_stats"]
    if build_stats:
        report["kd_stats"] = {
            "global_kd_avg": mean([b["global_kd"] for b in build_stats]),
            "global_pass_avg": mean([b["global_pass"] for b in build_stats]),
            "global_fail_avg": mean([b["global_fail"] for b in build_stats]),
            "local_kd_avg": mean([b["local_kd"] for b in build_stats]),
            "local_pass_avg": mean([b["local_pass"] for b in build_stats]),
            "local_fail_avg": mean([b["local_fail"] for b in build_stats]),
        }
        # 计算通过率
        total_global = sum([b["global_kd"] for b in build_stats])
        total_global_pass = sum([b["global_pass"] for b in build_stats])
        total_local = sum([b["local_kd"] for b in build_stats])
        total_local_pass = sum([b["local_pass"] for b in build_stats])

        report["kd_stats"]["global_pass_rate"] = total_global_pass / total_global * 100 if total_global > 0 else 0
        report["kd_stats"]["local_pass_rate"] = total_local_pass / total_local * 100 if total_local > 0 else 0

    # 6. 动态搜索半径统计
    if modes_iter0 and "thres_dist" in modes_iter0[0]:
        thres_dists = [m["thres_dist"] for m in modes_iter0]
        report["thres_dist_stats"] = {
            "avg": mean(thres_dists),
            "max": max(thres_dists),
            "min": min(thres_dists),
            "dynamic_count": sum(1 for t in thres_dists if t > 1.01),  # 动态调整的帧数
            "max_count": sum(1 for t in thres_dists if t >= 3.99),     # 达到最大值的帧数
        }

    # 7. 漂移诊断统计
    pose_deltas = residual_data.get("pose_delta", [])
    if pose_deltas:
        speeds = [p["speed"] for p in pose_deltas]
        deltaTs = [p["deltaT"] for p in pose_deltas]
        iters_list = [p["iters"] for p in pose_deltas]

        # 检测"静止"帧 (速度应该>0但优化器认为静止)
        # 车辆正常行驶速度约 50-60km/h ≈ 14-17 m/s
        # 如果速度 < 1 m/s 且连续多帧，可能是漂移导致的假静止
        low_speed_frames = [p for p in pose_deltas if p["speed"] < 1.0]
        zero_speed_frames = [p for p in pose_deltas if p["speed"] < 0.1]

        report["drift_diagnosis"] = {
            "speed_avg": mean(speeds),
            "speed_max": max(speeds),
            "speed_min": min(speeds),
            "deltaT_avg": mean(deltaTs),
            "deltaT_max": max(deltaTs),
            "iters_avg": mean(iters_list),
            "max_iters_count": sum(1 for i in iters_list if i >= 5),  # 达到最大迭代的帧数
            "low_speed_frames": len(low_speed_frames),    # 低速帧数 (<1 m/s)
            "zero_speed_frames": len(zero_speed_frames),  # 近零速帧数 (<0.1 m/s)
            "low_speed_details": low_speed_frames[:20],   # 详细信息 (前20帧)
        }

    return report


def print_residual_report(report: dict):
    """打印残差分析报告"""
    print("\n【残差统计】(iter 0)")
    print("-" * 70)

    if "candidates" in report and "kept" in report:
        print(f"{'特征类型':<10} {'候选 (avg/max/min)':<25} {'保留 (avg/max/min)':<25} {'保留率':<10}")
        print("-" * 70)

        for ftype in ["corner", "surf", "non"]:
            cand = report["candidates"][ftype]
            kept = report["kept"][ftype]
            rate = kept["avg"] / cand["avg"] * 100 if cand["avg"] > 0 else 0
            print(f"{ftype:<10} {cand['avg']:>6.0f} / {cand['max']:>4d} / {cand['min']:>4d}       "
                  f"{kept['avg']:>6.0f} / {kept['max']:>4d} / {kept['min']:>4d}       {rate:>5.1f}%")

    # 角点自适应模式
    if "modes" in report:
        print("\n【角点自适应模式】(iter 0)")
        print("-" * 70)
        print(f"{'模式':<15} {'帧数':<10} {'占比':<10} {'avg_global_kd':<15}")
        print("-" * 70)
        for mode, data in sorted(report["modes"].items()):
            print(f"{mode:<15} {data['count']:<10} {data['percent']:>5.1f}%     {data['avg_global_kd']:>8.1f}")

    # 全局/局部 KD 匹配率
    if "kd_stats" in report:
        kd = report["kd_stats"]
        print("\n【全局/局部 KD 匹配率】")
        print("-" * 70)
        print(f"全局 KD 尝试: {kd['global_kd_avg']:.0f} avg,  全局通过: {kd['global_pass_avg']:.0f} avg ({kd['global_pass_rate']:.1f}%)")
        print(f"局部 KD 尝试: {kd['local_kd_avg']:.0f} avg,  局部通过: {kd['local_pass_avg']:.0f} avg ({kd['local_pass_rate']:.1f}%)")

    # 低特征帧警告
    low_frames = report.get("low_feature_frames", [])
    local_only = report.get("local_only_frames", [])

    if low_frames or local_only:
        print("\n【⚠️ 风险帧警告】")
        print("-" * 70)

    if low_frames:
        print(f"\n低特征帧 (corner < 100): {len(low_frames)} 帧")
        print(f"{'帧号':<10} {'角点保留':<12} {'全局':<10} {'局部':<10} {'面点':<10} {'标记':<10}")
        # 只显示前20帧
        for k in low_frames[:20]:
            marker = "⚠️ 仅局部" if k["corner_global"] == 0 else ""
            print(f"Frame {k['frame']:<4} {k['corner']:<12} {k['corner_global']:<10} {k['corner_local']:<10} {k['surf']:<10} {marker}")
        if len(low_frames) > 20:
            print(f"... 还有 {len(low_frames) - 20} 帧")

    if local_only and not low_frames:  # 如果低特征帧已经显示了仅局部帧，就不重复
        # 过滤掉已在low_frames中显示的
        unique_local_only = [f for f in local_only if f["corner"] >= 100]
        if unique_local_only:
            print(f"\n仅局部角点帧 (global=0): {len(unique_local_only)} 帧")
            for k in unique_local_only[:10]:
                print(f"  Frame {k['frame']}: corner={k['corner']} (all local)")
            if len(unique_local_only) > 10:
                print(f"  ... 还有 {len(unique_local_only) - 10} 帧")

    # 动态搜索半径统计
    if "thres_dist_stats" in report:
        td = report["thres_dist_stats"]
        print("\n【动态搜索半径统计】")
        print("-" * 70)
        print(f"thres_dist: avg={td['avg']:.2f}m  max={td['max']:.2f}m  min={td['min']:.2f}m")
        print(f"动态调整帧数: {td['dynamic_count']}  达到最大值(4m)帧数: {td['max_count']}")

    # 漂移诊断
    if "drift_diagnosis" in report:
        dd = report["drift_diagnosis"]
        print("\n【🔍 漂移诊断】")
        print("-" * 70)
        print(f"速度: avg={dd['speed_avg']:.2f} m/s  max={dd['speed_max']:.2f} m/s  min={dd['speed_min']:.2f} m/s")
        print(f"位姿变化: avg={dd['deltaT_avg']:.4f}m  max={dd['deltaT_max']:.4f}m")
        print(f"迭代次数: avg={dd['iters_avg']:.1f}  达到最大迭代(5)帧数: {dd['max_iters_count']}")
        print(f"低速帧 (<1m/s): {dd['low_speed_frames']}  近零速帧 (<0.1m/s): {dd['zero_speed_frames']}")

        # 显示低速帧详情
        if dd['low_speed_frames'] > 0:
            print("\n低速帧详情 (前20帧):")
            print(f"{'帧号':<10} {'速度(m/s)':<12} {'deltaT(m)':<12} {'迭代次数':<10}")
            for p in dd['low_speed_details']:
                marker = "⚠️" if p["speed"] < 0.1 else ""
                print(f"Frame {p['frame']:<4} {p['speed']:<12.2f} {p['deltaT']:<12.4f} {p['iters']:<10} {marker}")


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
    parser.add_argument("--no-residuals", action="store_true", help="Skip residual analysis")
    args = parser.parse_args()

    summary, frame_intervals, frame_timestamps, residual_data = analyze_log(args.log)

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

    # 残差分析
    if not args.no_residuals and residual_data["candidates"]:
        residual_report = analyze_residuals(residual_data)
        print_residual_report(residual_report)

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
