#!/bin/bash
# LIO-Livox PoseEstimation GDB调试 - 跟踪Ceres优化详情
# 用法: ./debug_pe.sh

OUTPUT_FILE="/tmp/frame1_ceres_opi.txt"

echo "=========================================="
echo "  LIO-Livox 跟踪 Ceres 优化详情"
echo "=========================================="

if ! rostopic list &>/dev/null; then
    echo "[ERROR] roscore未运行，请先启动roscore"
    exit 1
fi

source ~/ws_livox/devel/setup.bash

# 设置参数
echo "[INFO] 设置ROS参数..."
rosparam load $(rospack find lio_livox)/config/horizon_params.yaml /PoseEstimation
rosparam set /PoseEstimation/IMU_Mode 2
rosparam set /PoseEstimation/filter_parameter_corner 0.2
rosparam set /PoseEstimation/filter_parameter_surf 0.4
rosparam set /PoseEstimation/Extrinsic_Tlb "[1.0, 0.0, 0.0, -0.05512, 0.0, 1.0, 0.0, -0.02226, 0.0, 0.0, 1.0, 0.0297, 0.0, 0.0, 0.0, 1.0]"

GDB_SCRIPT=$(mktemp /tmp/gdb_pe_XXXXXX.gdb)
cat > "$GDB_SCRIPT" << 'EOF'
set pagination off
set logging file /tmp/frame1_ceres_opi.txt
set logging overwrite on
set logging on

# 断点: Ceres优化完成后 (1482行 - ceres_solve_ms赋值)
# 此时 summary, cntCorner, cntSurf, cntNon 都在作用域内
break Estimator.cpp:1482

commands 1
    echo \n========== CERES OPTIMIZATION (frame_count=
    print frame_count
    echo , iterOpt=
    print iterOpt
    echo ) ==========\n
    
    echo [特征残差数量]\n
    print cntCorner
    print cntSurf
    print cntNon
    
    echo \n[Ceres Summary]\n
    print summary.num_successful_steps
    print summary.num_unsuccessful_steps
    print summary.initial_cost
    print summary.final_cost
    print summary.termination_type
    
    echo \n[优化前位姿 t_before_opti]\n
    print t_before_opti
    
    echo \n[当前帧位姿 lidarFrameList.back().P]\n
    print lidarFrameList.back().P
    
    echo \n========== END ==========\n
    continue
end

echo \n[GDB] 断点设置在 Estimator.cpp:1487\n
echo [GDB] 输出保存到 /tmp/frame1_ceres_opi.txt\n
echo [GDB] 输入 'run' 启动程序\n
EOF

echo "[INFO] 输出将保存到: $OUTPUT_FILE"
echo ""
echo "=========================================="
echo "  GDB启动后输入 'run'"
echo "  然后在另一终端播放rosbag"
echo "=========================================="
echo ""

gdb -x "$GDB_SCRIPT" --args $(rospack find lio_livox)/../../devel/lib/lio_livox/PoseEstimation __name:=PoseEstimation

rm -f "$GDB_SCRIPT"
