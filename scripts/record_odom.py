#!/usr/bin/env python3
"""
Record LIO-Livox odometry to TUM format for EVO evaluation.
增加实时性监控：帧间隔、处理延迟统计

Usage:
  1. Start recording: python3 record_odom.py
  2. Run LIO-Livox and play bag
  3. Ctrl+C to stop and save
  
Output: odom_traj_YYYYMMDD_HHMMSS.txt (TUM format)
"""

import rospy
from nav_msgs.msg import Odometry
import os
import sys
import time
from datetime import datetime

class OdomRecorder:
    def __init__(self):
        rospy.init_node('odom_recorder', anonymous=True)
        
        # Output file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        os.makedirs(self.output_dir, exist_ok=True)
        self.output_file = os.path.join(self.output_dir, f'odom_traj_{timestamp}.txt')
        
        self.poses = []
        self.count = 0
        
        # === 新增: 延迟和帧间隔统计 ===
        self.latencies = []        # 处理延迟 (系统时间 - 消息时间戳)
        self.intervals = []        # 帧间隔 (系统时间)
        self.last_wall_time = None
        self.first_msg_time = None
        self.last_msg_time = None
        
        # Subscribe to odometry
        self.sub = rospy.Subscriber('/livox_odometry_mapped', Odometry, self.odom_callback)
        
        rospy.loginfo(f"[OdomRecorder] Started. Output: {self.output_file}")
        rospy.loginfo("[OdomRecorder] Waiting for /livox_odometry_mapped ...")
        
    def odom_callback(self, msg):
        wall_time = time.time()  # 系统时间
        
        # Extract timestamp
        t = msg.header.stamp.to_sec()
        
        # === 新增: 计算延迟和帧间隔 ===
        latency_ms = (wall_time - t) * 1000.0
        self.latencies.append(latency_ms)
        
        if self.last_wall_time is not None:
            interval_ms = (wall_time - self.last_wall_time) * 1000.0
            self.intervals.append(interval_ms)
        self.last_wall_time = wall_time
        
        if self.first_msg_time is None:
            self.first_msg_time = t
        self.last_msg_time = t
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Extract quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # TUM format: timestamp tx ty tz qx qy qz qw
        self.poses.append(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}")
        
        self.count += 1
        # 每100帧显示实时统计
        if self.count % 100 == 0:
            recent_lat = self.latencies[-100:] if len(self.latencies) >= 100 else self.latencies
            avg_lat = sum(recent_lat) / len(recent_lat)
            max_lat = max(recent_lat)
            rospy.loginfo(f"[OdomRecorder] 帧:{self.count} | 延迟: 平均={avg_lat:.0f}ms 最大={max_lat:.0f}ms")
    
    def save(self):
        if not self.poses:
            rospy.logwarn("[OdomRecorder] No poses received!")
            return
            
        with open(self.output_file, 'w') as f:
            f.write('\n'.join(self.poses))
            f.write('\n')
        
        rospy.loginfo(f"[OdomRecorder] Saved {len(self.poses)} poses to {self.output_file}")
        
        # === 新增: 打印完整统计 ===
        self.print_stats()
    
    def print_stats(self):
        """打印延迟和帧间隔统计"""
        if len(self.poses) < 2:
            return
        
        # 基本信息
        first = self.poses[0].split()
        last = self.poses[-1].split()
        bag_duration = float(last[0]) - float(first[0])
        
        print("\n" + "=" * 65)
        print("📊 性能统计报告")
        print("=" * 65)
        print(f"  总帧数:     {len(self.poses)}")
        print(f"  Bag时长:    {bag_duration:.1f}s")
        print(f"  平均帧率:   {len(self.poses)/bag_duration:.1f} Hz")
        
        # 延迟统计
        if self.latencies:
            lat = sorted(self.latencies)
            n = len(lat)
            print(f"\n📈 处理延迟 (系统时间 - 消息时间戳):")
            print(f"  最小:   {lat[0]:8.1f} ms")
            print(f"  P50:    {lat[int(n*0.50)]:8.1f} ms")
            print(f"  平均:   {sum(lat)/n:8.1f} ms")
            print(f"  P95:    {lat[int(n*0.95)]:8.1f} ms")
            print(f"  P99:    {lat[int(n*0.99)]:8.1f} ms")
            print(f"  最大:   {lat[-1]:8.1f} ms")
            
            # 检测累积延迟
            if n > 200:
                first_100 = sum(self.latencies[:100]) / 100
                last_100 = sum(self.latencies[-100:]) / 100
                drift = last_100 - first_100
                print(f"\n  开始延迟: {first_100:.1f}ms → 结束延迟: {last_100:.1f}ms")
                if drift > 50:
                    print(f"  ⚠️  累积延迟: +{drift:.1f}ms (算法跟不上输入)")
                else:
                    print(f"  ✅ 无明显累积延迟")
        
        # 帧间隔统计
        if self.intervals:
            intv = sorted(self.intervals)
            n = len(intv)
            print(f"\n📊 帧间隔 (实际处理间隔):")
            print(f"  最小:   {intv[0]:8.1f} ms")
            print(f"  P50:    {intv[int(n*0.50)]:8.1f} ms")
            print(f"  平均:   {sum(intv)/n:8.1f} ms ({1000/(sum(intv)/n):.1f} Hz)")
            print(f"  P95:    {intv[int(n*0.95)]:8.1f} ms")
            print(f"  最大:   {intv[-1]:8.1f} ms")
        
        # 实时性评估
        if self.latencies:
            p95 = sorted(self.latencies)[int(len(self.latencies)*0.95)]
            print(f"\n🎯 实时性评估:")
            if p95 < 100:
                print(f"  ✅ 优秀 - P95延迟={p95:.1f}ms < 100ms")
            elif p95 < 150:
                print(f"  ⚠️  良好 - P95延迟={p95:.1f}ms < 150ms")
            elif p95 < 200:
                print(f"  ⚠️  及格 - P95延迟={p95:.1f}ms < 200ms")
            else:
                print(f"  ❌ 不合格 - P95延迟={p95:.1f}ms >= 200ms")
        
        print("=" * 65 + "\n")

def main():
    recorder = OdomRecorder()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.save()

if __name__ == '__main__':
    main()

