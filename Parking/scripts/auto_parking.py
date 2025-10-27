#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Automatic parking behavior for TurtleBot3 using LaserScan front distance.
Subscribes: /scan (sensor_msgs/LaserScan)
Publishes:  /cmd_vel (geometry_msgs/Twist)
Parameters:
  ~target_distance   (default: 0.25 m) distance to stop from an obstacle (e.g., wall/slot)
  ~approach_speed    (default: 0.08 m/s)
  ~backoff_distance  (default: 0.10 m) optional small backoff after initial contact
  ~scan_angle_deg    (default: 20) front cone half-angle used to compute min distance
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class AutoParking(object):
    def __init__(self):
        self.target = float(rospy.get_param("~target_distance", 0.25))
        self.vx = float(rospy.get_param("~approach_speed", 0.08))
        self.backoff = float(rospy.get_param("~backoff_distance", 0.10))
        self.scan_angle = float(rospy.get_param("~scan_angle_deg", 20.0))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.state = "approach"

    def scan_cb(self, msg):
        n = len(msg.ranges)
        center = n // 2
        width = int((self.scan_angle/180.0*math.pi) / msg.angle_increment)
        i0, i1 = max(0, center - width), min(n-1, center + width)
        front = [r for r in msg.ranges[i0:i1] if not math.isinf(r) and not math.isnan(r)]
        min_d = min(front) if front else float('inf')

        tw = Twist()
        if self.state == "approach":
            if min_d > self.target:
                tw.linear.x = self.vx
            else:
                tw.linear.x = 0.0
                self.state = "stop"
        elif self.state == "stop":
            tw.linear.x = 0.0

        self.cmd_pub.publish(tw)

def main():
    rospy.init_node("auto_parking")
    AutoParking()
    rospy.loginfo("auto_parking running...")
    rospy.spin()

if __name__ == "__main__":
    main()
