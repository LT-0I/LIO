#!/usr/bin/env python3
"""
Compare OPi trajectory with NUC baseline using EVO.
Usage:
  python3 compare_traj.py [opi_traj.txt]
  
If no argument, uses newest odom_traj_*.txt in logs/
"""

import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime

# Paths
SCRIPT_DIR = Path(__file__).parent
LOGS_DIR = SCRIPT_DIR.parent / 'logs'
BASELINE_DIR = Path('/home/orangepi/ws_livox/optimization_memory/baseline_data')
BASELINE_TRAJ = BASELINE_DIR / 'benchmark_traj_20251204_000419.txt'

def find_newest_traj():
    """Find newest odom_traj_*.txt in logs/"""
    if not LOGS_DIR.exists():
        return None
    
    traj_files = list(LOGS_DIR.glob('odom_traj_*.txt'))
    if not traj_files:
        return None
    
    # Sort by modification time
    traj_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return traj_files[0]

def clean_traj_file(filepath):
    """Clean trajectory file - remove incomplete lines"""
    clean_lines = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) == 8:  # TUM format: t x y z qx qy qz qw
                try:
                    [float(p) for p in parts]
                    clean_lines.append(line)
                except ValueError:
                    continue
    
    # Write cleaned file
    clean_path = filepath.parent / f"{filepath.stem}_clean.txt"
    with open(clean_path, 'w') as f:
        f.write('\n'.join(clean_lines))
        f.write('\n')
    
    return clean_path, len(clean_lines)

def run_evo_ape(ref_traj, est_traj, skip_seconds=5.0):
    """Run EVO APE and return RMSE"""
    
    # Clean trajectories
    print(f"📄 Reference: {ref_traj}")
    print(f"📄 Estimated: {est_traj}")
    
    ref_clean, ref_count = clean_traj_file(ref_traj)
    est_clean, est_count = clean_traj_file(est_traj)
    
    print(f"   Reference poses: {ref_count}")
    print(f"   Estimated poses: {est_count}")
    
    # Skip initial poses (first N seconds)
    if skip_seconds > 0:
        skip_ref = []
        skip_est = []
        
        with open(ref_clean, 'r') as f:
            lines = f.readlines()
            if lines:
                first_t = float(lines[0].split()[0])
                skip_ref = [l for l in lines if float(l.split()[0]) >= first_t + skip_seconds]
        
        with open(est_clean, 'r') as f:
            lines = f.readlines()
            if lines:
                first_t = float(lines[0].split()[0])
                skip_est = [l for l in lines if float(l.split()[0]) >= first_t + skip_seconds]
        
        # Write skipped versions
        ref_skip = ref_clean.parent / f"{ref_clean.stem}_skip.txt"
        est_skip = est_clean.parent / f"{est_clean.stem}_skip.txt"
        
        with open(ref_skip, 'w') as f:
            f.writelines(skip_ref)
        with open(est_skip, 'w') as f:
            f.writelines(skip_est)
        
        ref_clean = ref_skip
        est_clean = est_skip
        print(f"   Skipped first {skip_seconds}s: ref={len(skip_ref)}, est={len(skip_est)} poses")
    
    # Run EVO APE
    cmd = [
        'evo_ape', 'tum',
        str(ref_clean), str(est_clean),
        '-va',
        '--align', '--correct_scale'
    ]
    
    print(f"\n🔬 Running: {' '.join(cmd)}\n")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        
        # Parse output
        output = result.stdout + result.stderr
        print(output)
        
        # Extract RMSE
        rmse = None
        for line in output.split('\n'):
            if 'rmse' in line.lower():
                parts = line.split()
                for i, p in enumerate(parts):
                    if 'rmse' in p.lower() and i + 1 < len(parts):
                        try:
                            rmse = float(parts[i + 1])
                            break
                        except ValueError:
                            continue
        
        return rmse, output
        
    except subprocess.TimeoutExpired:
        print("❌ EVO timeout!")
        return None, "Timeout"
    except Exception as e:
        print(f"❌ EVO error: {e}")
        return None, str(e)

def main():
    print("=" * 60)
    print("🔬 LIO-Livox Trajectory Comparison")
    print("=" * 60)
    
    # Get OPi trajectory
    if len(sys.argv) > 1:
        opi_traj = Path(sys.argv[1])
    else:
        opi_traj = find_newest_traj()
    
    if not opi_traj or not opi_traj.exists():
        print("❌ No OPi trajectory found!")
        print(f"   Looking in: {LOGS_DIR}")
        print("   Run record_odom.py first, or provide path as argument")
        sys.exit(1)
    
    # Check baseline
    if not BASELINE_TRAJ.exists():
        print(f"❌ Baseline not found: {BASELINE_TRAJ}")
        sys.exit(1)
    
    print(f"\n📍 OPi Trajectory: {opi_traj.name}")
    print(f"📍 NUC Baseline: {BASELINE_TRAJ.name}")
    
    # Run comparison
    rmse, output = run_evo_ape(BASELINE_TRAJ, opi_traj, skip_seconds=5.0)
    
    print("\n" + "=" * 60)
    if rmse is not None:
        print(f"📊 RMSE = {rmse:.4f} m")
        
        if rmse < 1.0:
            print("✅ 精度优秀 (<1m)")
        elif rmse < 10.0:
            print("⚠️ 精度一般 (1-10m)")
        elif rmse < 100.0:
            print("❌ 精度较差 (10-100m)")
        else:
            print("🚨 严重漂移 (>100m)")
    else:
        print("❌ 无法计算RMSE")
    print("=" * 60)

if __name__ == '__main__':
    main()

