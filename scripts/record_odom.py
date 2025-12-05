#!/usr/bin/env python3
"""
Record LIO-Livox odometry to TUM format for EVO evaluation.
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
        
        # Subscribe to odometry
        self.sub = rospy.Subscriber('/livox_odometry_mapped', Odometry, self.odom_callback)
        
        rospy.loginfo(f"[OdomRecorder] Started. Output: {self.output_file}")
        rospy.loginfo("[OdomRecorder] Waiting for /livox_odometry_mapped ...")
        
    def odom_callback(self, msg):
        # Extract timestamp
        t = msg.header.stamp.to_sec()
        
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
        if self.count % 100 == 0:
            rospy.loginfo(f"[OdomRecorder] Received {self.count} poses")
    
    def save(self):
        if not self.poses:
            rospy.logwarn("[OdomRecorder] No poses received!")
            return
            
        with open(self.output_file, 'w') as f:
            f.write('\n'.join(self.poses))
            f.write('\n')
        
        rospy.loginfo(f"[OdomRecorder] Saved {len(self.poses)} poses to {self.output_file}")
        
        # Also print summary
        if len(self.poses) > 0:
            first = self.poses[0].split()
            last = self.poses[-1].split()
            duration = float(last[0]) - float(first[0])
            rospy.loginfo(f"[OdomRecorder] Duration: {duration:.1f}s, FPS: {len(self.poses)/duration:.1f}")

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

