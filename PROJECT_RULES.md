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

1. 使用 `scripts/perf_analyzer.py` 分析性能和残差统计
2. 隧道场景重点关注:
   - `avg_global_kd` 变化趋势
   - `thres_dist` 是否动态调整
   - `low_feature` 模式占比
