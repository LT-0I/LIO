# Project Rules

## 编译规则

1. **始终使用 8 核并行编译**:
   ```bash
   cd /home/orangepi/ws_livox && catkin_make -DCATKIN_WHITELIST_PACKAGES="lio_livox" -j8
   ```

## 代码修改规则

1. **修改变量赋值时，必须全局搜索该变量的所有赋值点**:
   ```bash
   grep -n "变量名\s*=" 文件路径
   ```
   确保没有遗漏的覆盖点。

2. **修改前后对比检查**:
   - 修改逻辑变量时，确认所有分支的赋值行为
   - 特别注意循环内部、条件分支内部的重复赋值

3. **测试验证**:
   - 代码修改后，通过日志确认修改确实生效
   - 不要假设代码按预期工作，要用数据验证

## 日志分析

1. 使用 `scripts/auto_eval.py` 自动对比OPi与NUC基线并归档：
   ```bash
   python3 scripts/auto_eval.py              # 分析并归档
   python3 scripts/auto_eval.py --no-archive # 仅分析，不归档
   python3 scripts/auto_eval.py --skip-init 10  # 跳过前10秒初始化
   ```
2. **EVO轨迹对比**: 默认跳过前5秒初始化期，使用 `--align` 和 `--correct_scale`
3. 隧道场景重点关注:
   - `avg_global_kd` 变化趋势
   - `thres_dist` 是否动态调整
   - `low_feature` 模式占比

---

## 实验归档规则 (重要!)

### 原则
- **每个 branch 只做一次实验**
- 实验完成后，日志必须归档到 `optimization_memory/branch_reports/`

### 归档流程

1. **运行实验**: `roslaunch` + `rosbag play`

2. **运行分析**: `python3 scripts/auto_eval.py`

3. **归档日志**: 分析完成后，将日志从 `src/LIO/logs/` 移动到归档目录
   ```
   optimization_memory/branch_reports/
   └── {branch_name}/                     # 以分支名命名，同分支覆盖
       ├── internal_stats_*.csv          # 内部统计CSV
       ├── benchmark_traj_*.txt          # TUM轨迹文件
       ├── evaluation_report.md          # auto_eval.py生成的对比报告
       └── 实验结论.md                    # 人工/AI编写的中文结论
   ```

4. **命名规范**:
   - 文件夹名: `{branch_name}` (仅分支名，同分支会覆盖)
   - 示例: `enhancepi3`, `master`, `optimize_kdtree`

5. **实验结论.md 内容模板**:
   ```markdown
   # 实验结论: {branch_name}
   
   ## 实验日期
   YYYY-MM-DD HH:MM
   
   ## 实验目标
   本次实验的优化目标...
   
   ## 关键发现
   1. ...
   2. ...
   
   ## 性能对比
   | 指标 | OPi | NUC | 比例 |
   |------|-----|-----|------|
   | ... | ... | ... | ... |
   
   ## 结论与下一步
   ...
   ```

6. **更新全局日志**: 在 `optimization_memory/GLOBAL_LOG.md` 中添加该branch的章节

### 目录结构
```
optimization_memory/
├── GLOBAL_LOG.md                    # 全局优化日志 (所有branch汇总)
├── baseline_data/                   # NUC黄金基准数据 (不可修改)
│   ├── baseline_internal_stats_*.csv
│   └── baseline_traj_*.txt
└── branch_reports/                  # 各branch实验归档 (同分支覆盖)
    ├── master/
    ├── enhancepi3/
    └── ...
```

---

## Git 工作流

1. **检查当前 branch**: `git branch --show-current`
2. **切换 branch 前**: 确保当前实验已归档
3. **Branch 命名**: 使用描述性名称，如 `optimize_kdtree`, `reduce_features`
