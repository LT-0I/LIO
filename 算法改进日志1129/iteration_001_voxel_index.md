# Iteration 001: VoxelIndex 体素索引优化

**日期**: 2025-11-29
**目标**: 使用 O(1) 体素哈希索引替代局部地图的 O(log N) KD-Tree 近邻搜索

---

## 1. 问题描述

在 `Estimator.cpp` 中，`processPointToLine`、`processPointToPlan`、`processPointToPlanVec`、`processNonFeatureICP` 四个函数频繁调用 KD-Tree 的 `nearestKSearch` 进行 5-NN 查询。每帧约 500-1500 个特征点，KD-Tree 查询是 `Residual build` 阶段的主要开销。

## 2. 解决方案

### 2.1 VoxelIndex 类设计

实现 O(1) 空间哈希索引：
- 将 3D 空间划分为固定大小的体素（默认 0.5m）
- 使用 `std::unordered_map<int64_t, vector<int>>` 存储每个体素内的点索引
- 查询时访问中心体素及其 26 邻域，收集候选点后排序取 top-k

### 2.2 修改的文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `include/utils/VoxelIndex.h` | 新建 | VoxelIndex 类声明 |
| `src/lio/VoxelIndex.cpp` | 新建 | VoxelIndex 类实现 |
| `include/Estimator/Estimator.h` | 修改 | 添加 VoxelIndex 成员 |
| `src/lio/Estimator.cpp` | 修改 | 替换局部 KD-Tree 查询 |
| `include/MapManager/Map_Manager.h` | 修改 | MapManagerConfig 添加配置 |
| `src/lio/PoseEstimation.cpp` | 修改 | 读取配置参数 |
| `config/horizon_params.yaml` | 修改 | 添加配置参数 |
| `CMakeLists.txt` | 修改 | 添加源文件 |

### 2.3 配置参数

```yaml
use_voxel_index_local: true    # 启用体素索引
voxel_index_resolution: 0.5    # 体素分辨率（米）
```

## 3. 性能测试结果

**测试环境**: OrangePi 5 MAX (RK3588, 8GB RAM)
**测试数据**: car隧道.bag
**VoxelIndex 状态**: 已启用 (use_voxel_index_local: True)

### 3.1 各阶段耗时统计

| 阶段 | 调用次数 | 平均耗时 | 最大耗时 |
|------|----------|----------|----------|
| Estimator::Estimate | 6863 | 89.70 ms | 871.39 ms |
| **Residual build** | 12975 | **20.24 ms** | 504.62 ms |
| MapManager snapshot | 6863 | 13.64 ms | 74.68 ms |
| MapManager update | 6864 | 11.39 ms | 37.29 ms |
| Ceres solve | 12975 | 9.97 ms | 64.31 ms |
| Marginalization | 6768 | 8.53 ms | 33.84 ms |
| RemoveDistortion | 6864 | 5.01 ms | 21.37 ms |
| IMU_PreIntegration | 6769 | 0.33 ms | 4.58 ms |

### 3.2 帧率统计

- 总帧数: 6862
- 平均帧间隔: 111.44 ms (~9 Hz)
- 最大帧间隔: 903.92 ms
- 最小帧间隔: 41.84 ms

## 4. 分析与结论

### 4.1 当前状态

VoxelIndex 已成功集成并运行。`Residual build` 平均耗时 20.24 ms，符合实时性要求。

### 4.2 观察

1. **Residual build 耗时合理**: 20 ms 平均值在可接受范围内
2. **帧率约 9 Hz**: 接近目标 10 Hz
3. **偶发延迟**: 最大 Residual build 达 504 ms，可能与地图更新同步或特征数量突增有关
4. **主要瓶颈**: `Estimator::Estimate` 总耗时 89.7 ms，包含多个子阶段

### 4.3 后续优化建议

1. **基线对比**: 需要关闭 VoxelIndex 运行对比测试，量化实际收益
2. **体素分辨率调优**: 可尝试 0.3m 或 0.8m 看是否有更好的平衡
3. **MapManager snapshot 优化**: 13.6 ms 占比较高，可考虑异步化

## 5. 回滚方法

如需回退到 KD-Tree：
```yaml
# config/horizon_params.yaml
use_voxel_index_local: false
```

---

**状态**: 待用户审批
**下一步**: 等待基线对比测试数据
