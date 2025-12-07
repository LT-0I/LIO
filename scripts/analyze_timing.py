#!/usr/bin/env python3
"""
生成中文计时统计报告：
- 自动选择 logs/@csv_logs 下最新的 time_log_*.csv / pose_timing_*.csv / scan_timing_*.csv
- 计算计时分布（数量、均值、P50、P95、P99、最大）
- 输出 Markdown 报告到同一目录 timing_report_*.md

注意：不运行节点，仅离线读取 CSV。
"""

import csv
import datetime
import os
from pathlib import Path
from typing import Dict, List, Tuple

PKG_PATH = Path(__file__).resolve().parents[1]
LOG_DIR_CANDIDATES = [
    PKG_PATH / "logs" / "@csv_logs",
    Path("/tmp/lio_livox_csv"),
]


def find_log_dir() -> Path:
    for p in LOG_DIR_CANDIDATES:
        if p.exists():
            return p
    return LOG_DIR_CANDIDATES[0]


def pick_latest(log_dir: Path, prefix: str) -> Path:
    candidates = sorted(log_dir.glob(f"{prefix}*.csv"), key=lambda f: f.stat().st_mtime, reverse=True)
    return candidates[0] if candidates else None


def percentiles(values: List[float], qs: Tuple[float, ...]) -> Dict[float, float]:
    if not values:
        return {q: 0.0 for q in qs}
    sorted_vals = sorted(values)
    n = len(sorted_vals)
    result = {}
    for q in qs:
        if n == 1:
            result[q] = sorted_vals[0]
            continue
        idx = min(int(q * (n - 1)), n - 1)
        result[q] = sorted_vals[idx]
    return result


def load_column(path: Path, column: str) -> List[float]:
    vals: List[float] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if column in row and row[column]:
                try:
                    vals.append(float(row[column]))
                except ValueError:
                    continue
    return vals


def basic_stats(values: List[float]) -> Dict[str, float]:
    if not values:
        return {k: 0.0 for k in ["count", "avg", "p50", "p95", "p99", "max"]}
    pct = percentiles(values, (0.50, 0.95, 0.99))
    return {
        "count": float(len(values)),
        "avg": sum(values) / len(values),
        "p50": pct[0.50],
        "p95": pct[0.95],
        "p99": pct[0.99],
        "max": max(values),
    }


def section(title: str, stats: Dict[str, float]) -> str:
    return (
        f"### {title}\n"
        f"- 样本数: {stats['count']:.0f}\n"
        f"- 均值:   {stats['avg']:.3f} ms\n"
        f"- P50:    {stats['p50']:.3f} ms\n"
        f"- P95:    {stats['p95']:.3f} ms\n"
        f"- P99:    {stats['p99']:.3f} ms\n"
        f"- 最大:   {stats['max']:.3f} ms\n"
    )


def main() -> None:
    log_dir = find_log_dir()
    time_csv = pick_latest(log_dir, "time_log_")
    pose_csv = pick_latest(log_dir, "pose_timing_")
    scan_csv = pick_latest(log_dir, "scan_timing_")

    report_lines = [
        "# 计时报告",
        f"- 日志目录: {log_dir}",
        f"- 生成时间: {datetime.datetime.now().isoformat()}",
    ]

    pose_status_line = "宏观指标：未找到 pose_timing_*（整帧均值 < 100ms 视为达标）"

    if pose_csv:
        pose_total = basic_stats(load_column(pose_csv, "frame_total_ms"))
        status = "达标 ✅" if pose_total["avg"] < 100.0 else "未达标 ⚠️"
        pose_status_line = (
            f"宏观指标（Pose整帧均值）: {pose_total['avg']:.3f} ms，目标 < 100 ms -> {status}"
        )
        report_lines.append(f"\n## PoseEstimation\n- 源文件: {pose_csv.name}\n")
        report_lines.append(section("frame_total_ms（整帧总耗时）", pose_total))
        undistort = basic_stats(load_column(pose_csv, "undistort_ms"))
        backend = basic_stats(load_column(pose_csv, "backend_ms"))
        publish = basic_stats(load_column(pose_csv, "publish_ms"))
        report_lines.append(section("undistort_ms（去畸变）", undistort))
        report_lines.append(section("backend_ms（后端总耗时）", backend))
        report_lines.append(section("publish_ms（发布）", publish))
    else:
        report_lines.append("\n## PoseEstimation\n- 源文件: 未找到\n")

    if scan_csv:
        scan_total = basic_stats(load_column(scan_csv, "total_ms"))
        report_lines.append(f"\n## ScanRegistration\n- 源文件: {scan_csv.name}\n")
        report_lines.append(section("total_ms（前端整帧）", scan_total))
        feat_stats = basic_stats(load_column(scan_csv, "feature_extract_ms"))
        report_lines.append(section("feature_extract_ms（特征提取）", feat_stats))
    else:
        report_lines.append("\n## ScanRegistration\n- 源文件: 未找到\n")

    if time_csv:
        time_total = basic_stats(load_column(time_csv, "total_ms"))
        report_lines.append(f"\n## Estimator 分解\n- 源文件: {time_csv.name}\n")
        report_lines.append(section("total_ms（估计整体）", time_total))
        for col, zh in [
            ("prep_map_ms", "prep_map_ms（准备地图）"),
            ("build_ms", "build_ms（残差构建）"),
            ("solve_ms", "solve_ms（求解）"),
            ("marg_ms", "marg_ms（边缘化）"),
        ]:
            report_lines.append(section(zh, basic_stats(load_column(time_csv, col))))
    else:
        report_lines.append("\n## Estimator 分解\n- 源文件: 未找到\n")

    # 将宏观状态放在报告头部
    report_lines.insert(3, f"- {pose_status_line}")

    report_path = log_dir / f"timing_report_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.md"
    with report_path.open("w", encoding="utf-8") as f:
        f.write("\n".join(report_lines) + "\n")


if __name__ == "__main__":
    main()

