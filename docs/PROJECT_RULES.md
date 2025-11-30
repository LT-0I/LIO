# LIO-Livox Optimization Protocol for OrangePi 5 MAX

## 1. Hardware & Environment Constraints
- **Device:** OrangePi 5 MAX (RK3588, 8-Core CPU, 16GB RAM).
- **Core Strategy:** "Space-Time Tradeoff". Utilize the 16GB RAM (Look-up tables, caching, pre-allocation) to reduce CPU load.
- **Compilation:** ALWAYS use 8 cores: `catkin_make -j8` (or specific package build).
- **Goal:** Improve robustness in degenerate scenarios (tunnels) without sacrificing real-time performance.

## 2. File Paths & Configuration
- **Config:** `src/LIO/config/horizon_params.yaml`
- **Logs Output:** `src/LIO/logs/`
- **Bag Files:** `/home/orangepi/Desktop/rosbags/`
- **Scripts:** `src/LIO/scripts/` (Contains `perf_analyzer.py` and `capture_pose_bt.sh`)
- **Doc Path:** `src/LIO/算法改进日志1129/`

## 3. The "Optimization Loop" (Standard Operating Procedure)
For every optimization iteration, you MUST follow this sequence strictly:

**Phase A: Code & Build**
1. Modify C++ code based on the optimization plan.
2. Compile: `source devel/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="lio_livox" -j8`
3. If build fails, fix and retry until success.

**Phase B: Execution & Data Collection**
1. Modify `horizon_params.yaml`: Set `log_feature_counts` and `log_module_timing` based on need.
2. Launch Node: `roslaunch lio_livox horizon.launch` (redirect output to `src/LIO/logs/<bag_name>.log`).
3. **Simultaneously:** Run memory monitor: `src/LIO/scripts/capture_pose_bt.sh`.
4. Play Rosbag: `rosbag play <bag_path>` (Ensure log name matches bag name).
5. Wait for bag to finish.

**Phase C: Analysis**
1. Run analysis: `python3 src/LIO/scripts/perf_analyzer.py <log_file>`.
2. Analyze the memory log from Step B3.

**Phase D: Documentation (Mandatory)**
Create/Update a Markdown file in `src/LIO/算法改进日志1129/`.
Structure for each entry:
   - **## Improvement Content:** What did you change?
   - **## Experiment Conclusion:** Data from `perf_analyzer` + Memory usage + Visual observation.
   - **## Next Steps:** Plan for the next iteration.

**Phase E: User Approval**
STOP and wait for user feedback before starting the next iteration.