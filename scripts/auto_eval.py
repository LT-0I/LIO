#!/usr/bin/env python3
"""
auto_eval.py - Automated LIO-Livox Benchmark Evaluation Script
Compares OrangePi (OPi) performance against NUC Golden Baseline.

Usage:
    python3 auto_eval.py

The script will:
1. Find the NEWEST log folder in logs/
2. Load NUC baseline data from optimization_memory/baseline_data/
3. Compare RMSE (using evo) and timing metrics (using pandas)
4. Output a Markdown summary table
"""

import os
import sys
import subprocess
import pandas as pd
import numpy as np
from pathlib import Path
from datetime import datetime
import glob
import tempfile


# ============== Configuration ==============
SCRIPT_DIR = Path(__file__).parent.resolve()
LIO_ROOT = SCRIPT_DIR.parent  # src/LIO/
WS_ROOT = LIO_ROOT.parent.parent  # ws_livox/

# Paths
LOGS_DIR = LIO_ROOT / "logs"
BASELINE_DIR = WS_ROOT / "optimization_memory" / "baseline_data"
BRANCH_REPORTS_DIR = WS_ROOT / "optimization_memory" / "branch_reports"

# NUC Baseline files (with timestamp in name)
BASELINE_STATS_PATTERN = "baseline_internal_stats_*.csv"
BASELINE_TRAJ_PATTERN = "baseline_traj_*.txt"

# EVO comparison settings
EVO_SKIP_FIRST_SECONDS = 5.0  # Skip first N seconds to handle initialization drift
EVO_USE_ALIGN = True          # Use --align flag for SE(3) alignment
EVO_CORRECT_SCALE = True      # Use --correct_scale flag


def get_current_git_branch() -> str:
    """Get current git branch name."""
    try:
        result = subprocess.run(
            ["git", "branch", "--show-current"],
            capture_output=True, text=True, timeout=5,
            cwd=LIO_ROOT
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except:
        pass
    return "unknown"


def archive_experiment(log_folder: Path, report_content: str, opi_stats: dict, nuc_stats: dict) -> Path:
    """
    Archive experiment logs to branch_reports folder.
    Uses branch name only (no timestamp) - overwrites existing archive for same branch.
    Returns the archive folder path.
    """
    import shutil
    
    branch_name = get_current_git_branch()
    timestamp = log_folder.name  # YYYYMMDD_HHMMSS format for reference
    
    # Archive folder uses branch name only (overwrites previous)
    archive_dir = BRANCH_REPORTS_DIR / branch_name
    
    # If archive exists, remove old files first
    if archive_dir.exists():
        print(f"   ⚠️ Overwriting existing archive: {archive_dir}")
        shutil.rmtree(archive_dir)
    
    archive_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy log files to archive
    for file in log_folder.iterdir():
        if file.is_file():
            shutil.copy2(file, archive_dir / file.name)
    
    # Save evaluation report
    report_file = archive_dir / "evaluation_report.md"
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(report_content)
    
    # Generate 实验结论.md template
    conclusion_file = archive_dir / "实验结论.md"
    conclusion_content = generate_conclusion_template(
        branch_name, timestamp, opi_stats, nuc_stats
    )
    with open(conclusion_file, 'w', encoding='utf-8') as f:
        f.write(conclusion_content)
    
    # Remove original log folder after successful archive
    try:
        shutil.rmtree(log_folder)
        print(f"   ✅ Original log folder removed: {log_folder}")
    except Exception as e:
        print(f"   ⚠️ Could not remove original folder: {e}")
    
    return archive_dir


def generate_conclusion_template(branch_name: str, timestamp: str, opi_stats: dict, nuc_stats: dict) -> str:
    """Generate 实验结论.md template with pre-filled data."""
    
    # Parse timestamp
    try:
        dt = datetime.strptime(timestamp, "%Y%m%d_%H%M%S")
        date_str = dt.strftime("%Y-%m-%d %H:%M")
    except:
        date_str = timestamp
    
    template = f"""# 实验结论: {branch_name}

## 实验日期
{date_str}

## 实验目标
<!-- 请填写本次实验的优化目标 -->


## 关键发现
<!-- 请填写关键发现 -->
1. 
2. 
3. 

## 性能对比

| 指标 | OPi | NUC | 比例/差异 |
|------|-----|-----|----------|
| 总帧数 | {opi_stats.get('total_frames', 'N/A')} | {nuc_stats.get('total_frames', 'N/A')} | - |
| 平均帧时间 | {opi_stats.get('avg_total_frame_time_ms', 0):.2f} ms | {nuc_stats.get('avg_total_frame_time_ms', 0):.2f} ms | {opi_stats.get('avg_total_frame_time_ms', 0) / max(nuc_stats.get('avg_total_frame_time_ms', 1), 0.001):.2f}x |
| 平均特征数 | {opi_stats.get('avg_total_features', 0):.0f} | {nuc_stats.get('avg_total_features', 0):.0f} | - |
| 早收敛率 | {opi_stats.get('converged_early_pct', 0):.1f}% | {nuc_stats.get('converged_early_pct', 0):.1f}% | - |

## 轨迹精度
<!-- EVO RMSE结果，请从evaluation_report.md复制 -->


## 问题与分析
<!-- 发现的问题及原因分析 -->


## 结论与下一步
<!-- 总结及后续优化方向 -->


---
*自动生成于 {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}*
"""
    return template


def find_newest_log_folder(logs_dir: Path) -> Path:
    """Find the newest timestamped log folder in logs_dir."""
    if not logs_dir.exists():
        raise FileNotFoundError(f"Logs directory not found: {logs_dir}")
    
    # List all directories with timestamp format YYYYMMDD_HHMMSS
    folders = []
    for item in logs_dir.iterdir():
        if item.is_dir() and len(item.name) == 15 and item.name[8] == '_':
            try:
                # Validate timestamp format
                datetime.strptime(item.name, "%Y%m%d_%H%M%S")
                folders.append(item)
            except ValueError:
                continue
    
    if not folders:
        raise FileNotFoundError(f"No valid log folders found in {logs_dir}")
    
    # Sort by timestamp (newest first)
    folders.sort(key=lambda x: x.name, reverse=True)
    return folders[0]


def find_baseline_files(baseline_dir: Path):
    """Find NUC baseline CSV and TUM trajectory files."""
    if not baseline_dir.exists():
        raise FileNotFoundError(f"Baseline directory not found: {baseline_dir}")
    
    # Find stats CSV
    stats_files = list(baseline_dir.glob(BASELINE_STATS_PATTERN))
    if not stats_files:
        raise FileNotFoundError(f"No baseline stats CSV found matching {BASELINE_STATS_PATTERN}")
    stats_file = sorted(stats_files)[-1]  # Use newest if multiple
    
    # Find trajectory TUM
    traj_files = list(baseline_dir.glob(BASELINE_TRAJ_PATTERN))
    if not traj_files:
        raise FileNotFoundError(f"No baseline trajectory found matching {BASELINE_TRAJ_PATTERN}")
    traj_file = sorted(traj_files)[-1]  # Use newest if multiple
    
    return stats_file, traj_file


def find_opi_files(log_folder: Path):
    """Find OPi internal stats CSV and TUM trajectory files in log folder."""
    # Find stats CSV
    stats_files = list(log_folder.glob("internal_stats_*.csv"))
    if not stats_files:
        raise FileNotFoundError(f"No internal_stats CSV found in {log_folder}")
    stats_file = stats_files[0]
    
    # Find trajectory TUM
    traj_files = list(log_folder.glob("benchmark_traj_*.txt"))
    if not traj_files:
        raise FileNotFoundError(f"No benchmark_traj TUM file found in {log_folder}")
    traj_file = traj_files[0]
    
    return stats_file, traj_file


def clean_tum_trajectory(traj_path: Path, skip_first_seconds: float = 0.0) -> Path:
    """
    Clean TUM trajectory file by removing incomplete lines and trailing spaces.
    Optionally skip first N seconds to handle initialization drift.
    Returns path to cleaned temporary file.
    """
    cleaned_lines = []
    first_timestamp = None
    skipped_count = 0
    
    with open(traj_path, 'r') as f:
        for line in f:
            line = line.rstrip()  # Remove trailing whitespace/newlines
            if not line:  # Skip empty lines
                continue
            if line.startswith('#'):  # Keep comments
                cleaned_lines.append(line)
                continue
            # Check if line has exactly 8 space-separated values
            parts = line.split()
            if len(parts) == 8:
                try:
                    timestamp = float(parts[0])
                    # Track first timestamp
                    if first_timestamp is None:
                        first_timestamp = timestamp
                    # Skip lines within the initialization period
                    if skip_first_seconds > 0 and (timestamp - first_timestamp) < skip_first_seconds:
                        skipped_count += 1
                        continue
                    cleaned_lines.append(line)
                except ValueError:
                    print(f"   ⚠️ Skipping line with invalid timestamp: {line[:50]}...")
            else:
                print(f"   ⚠️ Skipping malformed line ({len(parts)} fields): {line[:50]}...")
    
    if skipped_count > 0:
        print(f"   ℹ️ Skipped {skipped_count} lines in first {skip_first_seconds:.1f}s (initialization)")
    
    # Write to temp file
    import tempfile
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False)
    temp_file.write('\n'.join(cleaned_lines) + '\n')
    temp_file.close()
    return Path(temp_file.name)


def calculate_evo_rmse(ref_traj: Path, est_traj: Path, skip_first_seconds: float = None) -> dict:
    """
    Calculate trajectory RMSE using evo_ape tool.
    Skips first N seconds to handle initialization drift.
    Returns dict with rmse, mean, median, std, min, max.
    """
    if skip_first_seconds is None:
        skip_first_seconds = EVO_SKIP_FIRST_SECONDS
    
    result = {
        "rmse": float("nan"),
        "mean": float("nan"),
        "median": float("nan"),
        "std": float("nan"),
        "min": float("nan"),
        "max": float("nan"),
        "skipped_seconds": skip_first_seconds,
        "error": None
    }
    
    try:
        # Clean trajectory files (remove incomplete lines, skip initialization period)
        clean_ref = clean_tum_trajectory(ref_traj, skip_first_seconds)
        clean_est = clean_tum_trajectory(est_traj, skip_first_seconds)
        
        # Build evo_ape command
        cmd = ["evo_ape", "tum", str(clean_ref), str(clean_est)]
        if EVO_USE_ALIGN:
            cmd.append("--align")
        if EVO_CORRECT_SCALE:
            cmd.append("--correct_scale")
        cmd.append("--verbose")
        
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60
        )
        
        if proc.returncode != 0:
            result["error"] = f"evo_ape failed: {proc.stderr}"
            return result
        
        # Parse output for metrics
        output = proc.stdout + proc.stderr
        
        for line in output.split('\n'):
            line = line.strip()
            if line.startswith("rmse"):
                parts = line.split()
                if len(parts) >= 2:
                    result["rmse"] = float(parts[1])
            elif line.startswith("mean"):
                parts = line.split()
                if len(parts) >= 2:
                    result["mean"] = float(parts[1])
            elif line.startswith("median"):
                parts = line.split()
                if len(parts) >= 2:
                    result["median"] = float(parts[1])
            elif line.startswith("std"):
                parts = line.split()
                if len(parts) >= 2:
                    result["std"] = float(parts[1])
            elif line.startswith("min"):
                parts = line.split()
                if len(parts) >= 2:
                    result["min"] = float(parts[1])
            elif line.startswith("max"):
                parts = line.split()
                if len(parts) >= 2:
                    result["max"] = float(parts[1])
                    
    except subprocess.TimeoutExpired:
        result["error"] = "evo_ape timed out"
    except FileNotFoundError:
        result["error"] = "evo_ape not found - please install: pip install evo"
    except Exception as e:
        result["error"] = str(e)
    finally:
        # Cleanup temp files
        try:
            if 'clean_ref' in dir() and clean_ref.exists():
                clean_ref.unlink()
            if 'clean_est' in dir() and clean_est.exists():
                clean_est.unlink()
        except:
            pass
    
    return result


def load_and_analyze_stats(csv_path: Path) -> dict:
    """Load internal stats CSV and calculate comprehensive metrics."""
    df = pd.read_csv(csv_path)
    
    # Check if data has valid optimization metrics (total_features > 0)
    df_with_features = df[df['total_features'] > 0]
    
    # If no optimization data, use all rows but flag it
    if len(df_with_features) == 0:
        df_valid = df  # Use all data
        instrumentation_complete = False
        print(f"   ⚠️ WARNING: CSV has no optimization metrics (total_features=0)")
        print(f"      This indicates incomplete C++ instrumentation.")
        print(f"      Available data: raw_cloud_size, imu_msg_count, timestamps")
    else:
        df_valid = df_with_features
        instrumentation_complete = True
    
    if len(df_valid) == 0:
        return {"error": "No data rows found in CSV"}
    
    # Helper function to safely get column mean
    def safe_mean(col_name, default=0.0):
        if col_name in df_valid.columns:
            return df_valid[col_name].mean()
        return default
    
    def safe_max(col_name, default=0.0):
        if col_name in df_valid.columns:
            return df_valid[col_name].max()
        return default
    
    def safe_quantile(col_name, q, default=0.0):
        if col_name in df_valid.columns:
            return df_valid[col_name].quantile(q)
        return default
    
    def safe_sum(col_name, default=0):
        if col_name in df_valid.columns:
            return df_valid[col_name].sum()
        return default
    
    # ============== Basic Frame Info ==============
    metrics = {
        "total_frames": len(df_valid),
        "duration_sec": df_valid['timestamp'].max() - df_valid['timestamp'].min() if 'timestamp' in df_valid.columns else 0,
        "instrumentation_complete": instrumentation_complete,
    }
    
    # ============== Timing Metrics (Speed Analysis) ==============
    metrics.update({
        # Total frame time
        "avg_total_frame_time_ms": safe_mean('total_frame_time_ms'),
        "max_total_frame_time_ms": safe_max('total_frame_time_ms'),
        "p95_total_frame_time_ms": safe_quantile('total_frame_time_ms', 0.95),
        "p99_total_frame_time_ms": safe_quantile('total_frame_time_ms', 0.99),
        
        # Time breakdown
        "avg_preprocess_time_ms": safe_mean('preprocess_time_ms'),
        "avg_kdtree_build_time_ms": safe_mean('kdtree_build_time_ms'),
        "avg_feature_extract_time_ms": safe_mean('feature_extract_time_ms'),
        "avg_feature_match_time_ms": safe_mean('feature_match_time_ms'),
        "avg_optimization_time_ms": safe_mean('optimization_time_ms'),
        "avg_map_update_time_ms": safe_mean('map_update_time_ms'),
        "avg_publish_time_ms": safe_mean('publish_time_ms'),
        
        # P95 for key timings
        "p95_optimization_time_ms": safe_quantile('optimization_time_ms', 0.95),
        "p95_preprocess_time_ms": safe_quantile('preprocess_time_ms', 0.95),
    })
    
    # ============== Feature Count Metrics (Accuracy Impact) ==============
    metrics.update({
        "avg_corner_features": safe_mean('corner_features'),
        "avg_surf_features": safe_mean('surf_features'),
        "avg_nonfeature_points": safe_mean('nonfeature_points'),
        "avg_total_features": safe_mean('total_features'),
        
        # Feature source breakdown
        "avg_corner_from_map": safe_mean('corner_from_map'),
        "avg_corner_from_local": safe_mean('corner_from_local'),
        "avg_surf_from_map": safe_mean('surf_from_map'),
        "avg_surf_from_local": safe_mean('surf_from_local'),
        
        # Feature ratio (map vs local)
        "corner_map_ratio": safe_mean('corner_from_map') / max(safe_mean('corner_features'), 1) * 100,
        "surf_map_ratio": safe_mean('surf_from_map') / max(safe_mean('surf_features'), 1) * 100,
    })
    
    # ============== Point Cloud Metrics (Input Size) ==============
    metrics.update({
        "avg_raw_cloud_size": safe_mean('raw_cloud_size'),
        "avg_downsampled_corner": safe_mean('downsampled_corner'),
        "avg_downsampled_surf": safe_mean('downsampled_surf'),
        "avg_downsampled_nonfeature": safe_mean('downsampled_nonfeature'),
    })
    
    # ============== Map Metrics (Memory/Complexity) ==============
    metrics.update({
        "avg_map_corner_size": safe_mean('map_corner_size'),
        "avg_map_surf_size": safe_mean('map_surf_size'),
        "avg_local_corner_size": safe_mean('local_corner_size'),
        "avg_local_surf_size": safe_mean('local_surf_size'),
        "max_map_corner_size": safe_max('map_corner_size'),
        "max_map_surf_size": safe_max('map_surf_size'),
        "max_local_corner_size": safe_max('local_corner_size'),
        "max_local_surf_size": safe_max('local_surf_size'),
    })
    
    # ============== IMU Metrics ==============
    metrics.update({
        "avg_imu_msg_count": safe_mean('imu_msg_count'),
        "avg_imu_dt_sec": safe_mean('imu_dt_sec'),
    })
    
    # ============== Optimization Metrics (Accuracy Quality) ==============
    metrics.update({
        "avg_outer_iterations": safe_mean('outer_iterations'),
        "avg_ceres_iterations": safe_mean('ceres_iterations'),
        "avg_initial_cost": safe_mean('initial_cost'),
        "avg_final_cost": safe_mean('final_cost'),
        "avg_cost_reduction_pct": (1 - safe_mean('final_cost') / max(safe_mean('initial_cost'), 1e-10)) * 100,
        "avg_delta_rotation_deg": safe_mean('delta_rotation_deg'),
        "avg_delta_translation_m": safe_mean('delta_translation_m'),
        "converged_early_pct": (safe_sum('converged_early') / len(df_valid)) * 100,
    })
    
    # ============== Feature Error Metrics (Matching Quality) ==============
    metrics.update({
        "avg_corner_error": safe_mean('avg_corner_error'),
        "avg_surf_error": safe_mean('avg_surf_error'),
        "avg_nonfeature_error": safe_mean('avg_nonfeature_error'),
        "max_corner_error": safe_max('avg_corner_error'),
        "max_surf_error": safe_max('avg_surf_error'),
    })
    
    # ============== Derived Metrics ==============
    total_time = metrics["avg_total_frame_time_ms"]
    if total_time > 0:
        metrics["pct_preprocess"] = metrics["avg_preprocess_time_ms"] / total_time * 100
        metrics["pct_optimization"] = metrics["avg_optimization_time_ms"] / total_time * 100
        metrics["pct_other"] = 100 - metrics["pct_preprocess"] - metrics["pct_optimization"]
    else:
        metrics["pct_preprocess"] = 0
        metrics["pct_optimization"] = 0
        metrics["pct_other"] = 0
    
    # Frame rate capability
    if total_time > 0:
        metrics["theoretical_fps"] = 1000.0 / total_time
    else:
        metrics["theoretical_fps"] = 0
    
    return metrics


def format_diff(opi_val, nuc_val, unit="", lower_is_better=True):
    """Format a comparison value with diff and color indicator."""
    if pd.isna(opi_val) or pd.isna(nuc_val):
        return f"{opi_val:.2f}{unit} (N/A)"
    
    diff = opi_val - nuc_val
    diff_pct = (diff / nuc_val * 100) if nuc_val != 0 else 0
    
    # Determine if this is good or bad
    if lower_is_better:
        is_good = diff <= 0
    else:
        is_good = diff >= 0
    
    indicator = "✅" if is_good else "⚠️"
    sign = "+" if diff > 0 else ""
    
    return f"{opi_val:.2f}{unit} ({sign}{diff_pct:.1f}%) {indicator}"


def generate_markdown_report(
    opi_stats: dict, 
    nuc_stats: dict,
    opi_evo: dict,
    nuc_evo: dict,
    opi_folder: str,
    nuc_folder: str
) -> str:
    """Generate comprehensive Markdown report comparing OPi vs NUC."""
    
    def get_ratio(opi_val, nuc_val):
        """Calculate ratio, handling edge cases."""
        if pd.isna(opi_val) or pd.isna(nuc_val) or nuc_val == 0:
            return float('nan')
        return opi_val / nuc_val
    
    def get_diff_pct(opi_val, nuc_val):
        """Calculate percentage difference."""
        if pd.isna(opi_val) or pd.isna(nuc_val) or nuc_val == 0:
            return float('nan')
        return (opi_val - nuc_val) / nuc_val * 100
    
    report = []
    report.append("# 🔬 LIO-Livox Benchmark Comparison Report")
    report.append("")
    report.append(f"**OrangePi Log**: `{opi_folder}`")
    report.append(f"**NUC Baseline**: `{nuc_folder}`")
    report.append(f"**Generated**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("")
    
    # === Basic Info ===
    report.append("## 📋 Basic Info")
    report.append("")
    
    # Check instrumentation status
    opi_instr = opi_stats.get('instrumentation_complete', True)
    nuc_instr = nuc_stats.get('instrumentation_complete', True)
    
    if not opi_instr:
        report.append("> ⚠️ **WARNING**: OPi CSV data is incomplete (optimization metrics are zeros).")
        report.append("> This indicates the C++ instrumentation code needs to be fixed.")
        report.append("> Trajectory data IS available for comparison.")
        report.append("")
    
    report.append("| Metric | OPi | NUC |")
    report.append("|--------|-----|-----|")
    report.append(f"| Total Frames | {opi_stats.get('total_frames', 'N/A')} | {nuc_stats.get('total_frames', 'N/A')} |")
    report.append(f"| Duration | {opi_stats.get('duration_sec', 0):.1f} sec | {nuc_stats.get('duration_sec', 0):.1f} sec |")
    report.append(f"| Theoretical FPS | {opi_stats.get('theoretical_fps', 0):.1f} Hz | {nuc_stats.get('theoretical_fps', 0):.1f} Hz |")
    report.append(f"| Instrumentation | {'✅ Complete' if opi_instr else '❌ Incomplete'} | {'✅ Complete' if nuc_instr else '❌ Incomplete'} |")
    report.append("")
    
    # === Trajectory Accuracy (EVO) ===
    report.append("## 📍 Trajectory Accuracy (EVO APE)")
    report.append("")
    skip_sec = opi_evo.get('skipped_seconds', 0)
    if skip_sec > 0:
        report.append(f"> ℹ️ Skipped first {skip_sec:.1f}s for initialization stabilization")
        report.append("")
    report.append("| Metric | OPi | NUC | Diff |")
    report.append("|--------|-----|-----|------|")
    
    if opi_evo.get("error"):
        report.append(f"| RMSE | ⚠️ Error: {opi_evo['error']} | - | - |")
    elif nuc_evo.get("error"):
        report.append(f"| RMSE | {opi_evo['rmse']:.4f} m | ⚠️ Error: {nuc_evo['error']} | - |")
    else:
        opi_rmse = opi_evo['rmse']
        nuc_rmse = nuc_evo['rmse']
        diff = opi_rmse - nuc_rmse
        diff_pct = (diff / nuc_rmse * 100) if nuc_rmse != 0 else 0
        indicator = "✅" if diff <= 0.001 else ("⚠️" if diff <= 0.01 else "❌")
        report.append(f"| RMSE | {opi_rmse:.4f} m | {nuc_rmse:.4f} m | {diff:+.4f} m ({diff_pct:+.1f}%) {indicator} |")
        
        opi_mean = opi_evo['mean']
        nuc_mean = nuc_evo['mean']
        diff = opi_mean - nuc_mean
        report.append(f"| Mean | {opi_mean:.4f} m | {nuc_mean:.4f} m | {diff:+.4f} m |")
        
        opi_max = opi_evo['max']
        nuc_max = nuc_evo['max']
        diff = opi_max - nuc_max
        report.append(f"| Max | {opi_max:.4f} m | {nuc_max:.4f} m | {diff:+.4f} m |")
    
    report.append("")
    
    # === Timing Performance ===
    report.append("## ⏱️ Timing Performance")
    report.append("")
    report.append("### Overall Frame Time")
    report.append("")
    report.append("| Metric | OPi | NUC | Ratio | Status |")
    report.append("|--------|-----|-----|-------|--------|")
    
    timing_metrics = [
        ('avg_total_frame_time_ms', 'Avg Frame Time', 'ms', 2.0, 3.0),
        ('p95_total_frame_time_ms', 'P95 Frame Time', 'ms', 2.5, 4.0),
        ('p99_total_frame_time_ms', 'P99 Frame Time', 'ms', 3.0, 5.0),
        ('max_total_frame_time_ms', 'Max Frame Time', 'ms', None, None),
    ]
    
    for key, name, unit, good_thresh, warn_thresh in timing_metrics:
        opi_val = opi_stats.get(key, float('nan'))
        nuc_val = nuc_stats.get(key, float('nan'))
        ratio = get_ratio(opi_val, nuc_val)
        if good_thresh and warn_thresh:
            status = "✅" if ratio < good_thresh else ("⚠️" if ratio < warn_thresh else "❌")
        else:
            status = "-"
        report.append(f"| {name} | {opi_val:.2f} {unit} | {nuc_val:.2f} {unit} | {ratio:.2f}x | {status} |")
    
    report.append("")
    report.append("### Time Breakdown (Average)")
    report.append("")
    report.append("| Stage | OPi (ms) | NUC (ms) | OPi % | NUC % | Ratio |")
    report.append("|-------|----------|----------|-------|-------|-------|")
    
    breakdown_metrics = [
        ('avg_preprocess_time_ms', 'Preprocess'),
        ('avg_kdtree_build_time_ms', 'KD-Tree Build'),
        ('avg_feature_match_time_ms', 'Feature Match'),
        ('avg_optimization_time_ms', 'Optimization'),
        ('avg_map_update_time_ms', 'Map Update'),
        ('avg_publish_time_ms', 'Publish'),
    ]
    
    opi_total = opi_stats.get('avg_total_frame_time_ms', 1)
    nuc_total = nuc_stats.get('avg_total_frame_time_ms', 1)
    
    for key, name in breakdown_metrics:
        opi_val = opi_stats.get(key, 0)
        nuc_val = nuc_stats.get(key, 0)
        opi_pct = (opi_val / opi_total * 100) if opi_total > 0 else 0
        nuc_pct = (nuc_val / nuc_total * 100) if nuc_total > 0 else 0
        ratio = get_ratio(opi_val, nuc_val)
        report.append(f"| {name} | {opi_val:.2f} | {nuc_val:.2f} | {opi_pct:.1f}% | {nuc_pct:.1f}% | {ratio:.2f}x |")
    
    report.append("")
    
    # === Feature Statistics ===
    report.append("## 📊 Feature Statistics")
    report.append("")
    report.append("### Feature Counts")
    report.append("")
    report.append("| Metric | OPi | NUC | Diff % |")
    report.append("|--------|-----|-----|--------|")
    
    feature_metrics = [
        ('avg_total_features', 'Total Features'),
        ('avg_corner_features', 'Corner Features'),
        ('avg_surf_features', 'Surf Features'),
        ('avg_nonfeature_points', 'Non-Feature Points'),
    ]
    
    for key, name in feature_metrics:
        opi_val = opi_stats.get(key, 0)
        nuc_val = nuc_stats.get(key, 0)
        diff_pct = get_diff_pct(opi_val, nuc_val)
        report.append(f"| {name} | {opi_val:.0f} | {nuc_val:.0f} | {diff_pct:+.1f}% |")
    
    report.append("")
    report.append("### Feature Source (Map vs Local)")
    report.append("")
    report.append("| Source | OPi | NUC |")
    report.append("|--------|-----|-----|")
    
    report.append(f"| Corner from Map | {opi_stats.get('avg_corner_from_map', 0):.0f} ({opi_stats.get('corner_map_ratio', 0):.1f}%) | {nuc_stats.get('avg_corner_from_map', 0):.0f} ({nuc_stats.get('corner_map_ratio', 0):.1f}%) |")
    report.append(f"| Corner from Local | {opi_stats.get('avg_corner_from_local', 0):.0f} | {nuc_stats.get('avg_corner_from_local', 0):.0f} |")
    report.append(f"| Surf from Map | {opi_stats.get('avg_surf_from_map', 0):.0f} ({opi_stats.get('surf_map_ratio', 0):.1f}%) | {nuc_stats.get('avg_surf_from_map', 0):.0f} ({nuc_stats.get('surf_map_ratio', 0):.1f}%) |")
    report.append(f"| Surf from Local | {opi_stats.get('avg_surf_from_local', 0):.0f} | {nuc_stats.get('avg_surf_from_local', 0):.0f} |")
    
    report.append("")
    
    # === Point Cloud & Map Size ===
    report.append("## 🗺️ Point Cloud & Map Size")
    report.append("")
    report.append("### Input Point Cloud")
    report.append("")
    report.append("| Metric | OPi | NUC | Diff % |")
    report.append("|--------|-----|-----|--------|")
    
    cloud_metrics = [
        ('avg_raw_cloud_size', 'Raw Cloud Size'),
        ('avg_downsampled_corner', 'Downsampled Corner'),
        ('avg_downsampled_surf', 'Downsampled Surf'),
    ]
    
    for key, name in cloud_metrics:
        opi_val = opi_stats.get(key, 0)
        nuc_val = nuc_stats.get(key, 0)
        diff_pct = get_diff_pct(opi_val, nuc_val)
        report.append(f"| {name} | {opi_val:.0f} | {nuc_val:.0f} | {diff_pct:+.1f}% |")
    
    report.append("")
    report.append("### Map Size (Avg / Max)")
    report.append("")
    report.append("| Map Type | OPi Avg | OPi Max | NUC Avg | NUC Max |")
    report.append("|----------|---------|---------|---------|---------|")
    
    report.append(f"| Global Corner | {opi_stats.get('avg_map_corner_size', 0):.0f} | {opi_stats.get('max_map_corner_size', 0):.0f} | {nuc_stats.get('avg_map_corner_size', 0):.0f} | {nuc_stats.get('max_map_corner_size', 0):.0f} |")
    report.append(f"| Global Surf | {opi_stats.get('avg_map_surf_size', 0):.0f} | {opi_stats.get('max_map_surf_size', 0):.0f} | {nuc_stats.get('avg_map_surf_size', 0):.0f} | {nuc_stats.get('max_map_surf_size', 0):.0f} |")
    report.append(f"| Local Corner | {opi_stats.get('avg_local_corner_size', 0):.0f} | {opi_stats.get('max_local_corner_size', 0):.0f} | {nuc_stats.get('avg_local_corner_size', 0):.0f} | {nuc_stats.get('max_local_corner_size', 0):.0f} |")
    report.append(f"| Local Surf | {opi_stats.get('avg_local_surf_size', 0):.0f} | {opi_stats.get('max_local_surf_size', 0):.0f} | {nuc_stats.get('avg_local_surf_size', 0):.0f} | {nuc_stats.get('max_local_surf_size', 0):.0f} |")
    
    report.append("")
    
    # === Optimization Quality ===
    report.append("## 🎯 Optimization Quality")
    report.append("")
    report.append("### Convergence")
    report.append("")
    report.append("| Metric | OPi | NUC | Diff |")
    report.append("|--------|-----|-----|------|")
    
    report.append(f"| Avg Outer Iterations | {opi_stats.get('avg_outer_iterations', 0):.2f} | {nuc_stats.get('avg_outer_iterations', 0):.2f} | {opi_stats.get('avg_outer_iterations', 0) - nuc_stats.get('avg_outer_iterations', 0):+.2f} |")
    report.append(f"| Avg Ceres Iterations | {opi_stats.get('avg_ceres_iterations', 0):.1f} | {nuc_stats.get('avg_ceres_iterations', 0):.1f} | {opi_stats.get('avg_ceres_iterations', 0) - nuc_stats.get('avg_ceres_iterations', 0):+.1f} |")
    report.append(f"| Converged Early % | {opi_stats.get('converged_early_pct', 0):.1f}% | {nuc_stats.get('converged_early_pct', 0):.1f}% | {opi_stats.get('converged_early_pct', 0) - nuc_stats.get('converged_early_pct', 0):+.1f}% |")
    
    report.append("")
    report.append("### Cost Reduction")
    report.append("")
    report.append("| Metric | OPi | NUC |")
    report.append("|--------|-----|-----|")
    
    report.append(f"| Avg Initial Cost | {opi_stats.get('avg_initial_cost', 0):.2e} | {nuc_stats.get('avg_initial_cost', 0):.2e} |")
    report.append(f"| Avg Final Cost | {opi_stats.get('avg_final_cost', 0):.2e} | {nuc_stats.get('avg_final_cost', 0):.2e} |")
    report.append(f"| Cost Reduction | {opi_stats.get('avg_cost_reduction_pct', 0):.2f}% | {nuc_stats.get('avg_cost_reduction_pct', 0):.2f}% |")
    
    report.append("")
    report.append("### Pose Change per Iteration")
    report.append("")
    report.append("| Metric | OPi | NUC | Diff |")
    report.append("|--------|-----|-----|------|")
    
    report.append(f"| Avg ΔRotation | {opi_stats.get('avg_delta_rotation_deg', 0):.4f}° | {nuc_stats.get('avg_delta_rotation_deg', 0):.4f}° | {opi_stats.get('avg_delta_rotation_deg', 0) - nuc_stats.get('avg_delta_rotation_deg', 0):+.4f}° |")
    report.append(f"| Avg ΔTranslation | {opi_stats.get('avg_delta_translation_m', 0):.4f} m | {nuc_stats.get('avg_delta_translation_m', 0):.4f} m | {opi_stats.get('avg_delta_translation_m', 0) - nuc_stats.get('avg_delta_translation_m', 0):+.4f} m |")
    
    report.append("")
    
    # === Feature Matching Quality ===
    report.append("## 🔗 Feature Matching Quality")
    report.append("")
    report.append("| Error Type | OPi Avg | OPi Max | NUC Avg | NUC Max |")
    report.append("|------------|---------|---------|---------|---------|")
    
    report.append(f"| Corner Error | {opi_stats.get('avg_corner_error', 0):.4f} | {opi_stats.get('max_corner_error', 0):.4f} | {nuc_stats.get('avg_corner_error', 0):.4f} | {nuc_stats.get('max_corner_error', 0):.4f} |")
    report.append(f"| Surf Error | {opi_stats.get('avg_surf_error', 0):.4f} | {opi_stats.get('max_surf_error', 0):.4f} | {nuc_stats.get('avg_surf_error', 0):.4f} | {nuc_stats.get('max_surf_error', 0):.4f} |")
    report.append(f"| Non-Feature Error | {opi_stats.get('avg_nonfeature_error', 0):.4f} | - | {nuc_stats.get('avg_nonfeature_error', 0):.4f} | - |")
    
    report.append("")
    
    # === Summary ===
    report.append("## 📝 Summary")
    report.append("")
    
    # Timing ratio for summary
    opi_ft = opi_stats.get('avg_total_frame_time_ms', float('nan'))
    nuc_ft = nuc_stats.get('avg_total_frame_time_ms', float('nan'))
    ratio = get_ratio(opi_ft, nuc_ft)
    
    # Check if we have EVO results
    if not opi_evo.get("error") and not nuc_evo.get("error"):
        rmse_diff = opi_evo['rmse'] - nuc_evo['rmse']
        if abs(rmse_diff) < 0.001:
            report.append("- ✅ **Trajectory accuracy matches NUC baseline** (RMSE diff < 1mm)")
        elif abs(rmse_diff) < 0.01:
            report.append("- ⚠️ **Trajectory accuracy within acceptable range** (RMSE diff < 1cm)")
        else:
            report.append(f"- ❌ **Trajectory accuracy degraded** (RMSE diff = {rmse_diff*100:.1f}cm)")
    else:
        report.append("- ⚠️ **Trajectory comparison not available** (EVO error)")
    
    # Timing summary
    if not pd.isna(ratio):
        if ratio < 2.0:
            report.append(f"- ✅ **Timing performance acceptable** ({ratio:.1f}x slower than NUC)")
        elif ratio < 3.0:
            report.append(f"- ⚠️ **Timing needs optimization** ({ratio:.1f}x slower than NUC)")
        else:
            report.append(f"- ❌ **Timing significantly degraded** ({ratio:.1f}x slower than NUC)")
    
    # Feature matching summary
    opi_corner_err = opi_stats.get('avg_corner_error', 0)
    nuc_corner_err = nuc_stats.get('avg_corner_error', 0)
    if opi_corner_err > 0 and nuc_corner_err > 0:
        err_ratio = opi_corner_err / nuc_corner_err
        if err_ratio < 1.1:
            report.append(f"- ✅ **Feature matching quality similar** (corner error ratio: {err_ratio:.2f}x)")
        elif err_ratio < 1.5:
            report.append(f"- ⚠️ **Feature matching slightly degraded** (corner error ratio: {err_ratio:.2f}x)")
        else:
            report.append(f"- ❌ **Feature matching significantly degraded** (corner error ratio: {err_ratio:.2f}x)")
    
    # FPS summary
    opi_fps = opi_stats.get('theoretical_fps', 0)
    if opi_fps >= 10:
        report.append(f"- ✅ **Real-time capable** (theoretical {opi_fps:.1f} FPS)")
    elif opi_fps >= 5:
        report.append(f"- ⚠️ **Near real-time** (theoretical {opi_fps:.1f} FPS, target: 10 Hz)")
    else:
        report.append(f"- ❌ **Below real-time** (theoretical {opi_fps:.1f} FPS, target: 10 Hz)")
    
    report.append("")
    report.append("---")
    report.append("*Generated by auto_eval.py*")
    
    return "\n".join(report)


def main():
    global EVO_SKIP_FIRST_SECONDS
    
    import argparse
    parser = argparse.ArgumentParser(description="LIO-Livox Benchmark Evaluation")
    parser.add_argument("--no-archive", action="store_true", 
                        help="Skip archiving logs to branch_reports")
    parser.add_argument("--skip-init", type=float, default=EVO_SKIP_FIRST_SECONDS,
                        help=f"Skip first N seconds for initialization (default: {EVO_SKIP_FIRST_SECONDS})")
    args = parser.parse_args()
    
    # Update skip setting from argument
    EVO_SKIP_FIRST_SECONDS = args.skip_init
    
    print("=" * 60)
    print("🔬 LIO-Livox Auto Evaluation Script")
    print("=" * 60)
    print()
    
    # Show current branch
    branch_name = get_current_git_branch()
    print(f"📌 Current Git Branch: {branch_name}")
    print()
    
    try:
        # Step 1: Find newest OPi log folder
        print("📁 Finding newest log folder...")
        opi_folder = find_newest_log_folder(LOGS_DIR)
        print(f"   Found: {opi_folder.name}")
        
        # Step 2: Find OPi files
        print("📄 Loading OPi log files...")
        opi_stats_file, opi_traj_file = find_opi_files(opi_folder)
        print(f"   Stats: {opi_stats_file.name}")
        print(f"   Traj:  {opi_traj_file.name}")
        
        # Step 3: Find NUC baseline files
        print("📄 Loading NUC baseline files...")
        nuc_stats_file, nuc_traj_file = find_baseline_files(BASELINE_DIR)
        print(f"   Stats: {nuc_stats_file.name}")
        print(f"   Traj:  {nuc_traj_file.name}")
        
        # Step 4: Analyze internal stats
        print("📊 Analyzing internal statistics...")
        opi_stats = load_and_analyze_stats(opi_stats_file)
        nuc_stats = load_and_analyze_stats(nuc_stats_file)
        print(f"   OPi frames: {opi_stats.get('total_frames', 'N/A')}")
        print(f"   NUC frames: {nuc_stats.get('total_frames', 'N/A')}")
        
        # Step 5: Calculate trajectory RMSE with EVO
        print("📍 Calculating trajectory RMSE with EVO...")
        print("   (Note: EVO compares OPi trajectory against NUC baseline)")
        opi_evo = calculate_evo_rmse(nuc_traj_file, opi_traj_file)
        nuc_evo = {"rmse": 0.0, "mean": 0.0, "median": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}  # NUC is reference
        
        if opi_evo.get("error"):
            print(f"   ⚠️ EVO Error: {opi_evo['error']}")
        else:
            print(f"   RMSE: {opi_evo['rmse']:.4f} m")
        
        # Step 6: Generate report
        print()
        print("=" * 60)
        report = generate_markdown_report(
            opi_stats, nuc_stats,
            opi_evo, nuc_evo,
            opi_folder.name, nuc_stats_file.parent.name
        )
        print(report)
        
        # Step 7: Archive experiment (unless --no-archive)
        if not args.no_archive:
            print()
            print("=" * 60)
            print("📦 Archiving experiment to branch_reports...")
            archive_dir = archive_experiment(opi_folder, report, opi_stats, nuc_stats)
            print(f"   ✅ Archived to: {archive_dir}")
            print(f"   📝 Please edit: {archive_dir / '实验结论.md'}")
        else:
            # Just save report to original folder
            report_file = opi_folder / "evaluation_report.md"
            with open(report_file, 'w') as f:
                f.write(report)
            print()
            print(f"📝 Report saved to: {report_file}")
            print("   (Use without --no-archive to auto-archive)")
        
    except FileNotFoundError as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

