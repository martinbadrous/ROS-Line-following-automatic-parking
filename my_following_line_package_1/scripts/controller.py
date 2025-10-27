#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PID controller for line following (ROS Melodic / Python2).
Subscribes: /line_error (Float32), /line_visible (Bool)
Publishes:  /cmd_vel (geometry_msgs/Twist)
Parameters:
  ~kp, ~ki, ~kd        (PID gains; defaults: 0.004, 0.0, 0.002)
  ~max_ang             (max angular speed, default 1.5)
  ~lin_speed           (forward linear speed when line visible, default 0.15 m/s)
  ~lin_speed_search    (forward speed when searching, default 0.05 m/s)
  ~search_turn         (angular speed when searching, default 0.6)
"""
import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time

class PIDController(object):
    def __init__(self):
        self.kp = float(rospy.get_param("~kp", 0.004))
        self.ki = float(rospy.get_param("~ki", 0.0))
        self.kd = float(rospy.get_param("~kd", 0.002))
        self.max_ang = float(rospy.get_param("~max_ang", 1.5))
        self.lin_speed = float(rospy.get_param("~lin_speed", 0.15))
        self.lin_speed_search = float(rospy.get_param("~lin_speed_search", 0.05))
        self.search_turn = float(rospy.get_param("~search_turn", 0.6))

        self.error = 0.0
        self.visible = False
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_t = time.time()

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/line_error", Float32, self.err_cb)
        rospy.Subscriber("/line_visible", Bool, self.vis_cb)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)  # 20 Hz

    def err_cb(self, msg):
        self.error = float(msg.data)

    def vis_cb(self, msg):
        self.visible = bool(msg.data)

    def control_loop(self, event):
        now = time.time()
        dt = max(1e-3, now - self.prev_t)
        self.prev_t = now

        twist = Twist()

        if self.visible:
            # PID
            self.integral += self.error * dt
            derivative = (self.error - self.prev_error) / dt
            ang_z = self.kp * self.error + self.ki * self.integral + self.kd * derivative
            self.prev_error = self.error
            # clamp
            if ang_z > self.max_ang: ang_z = self.max_ang
            if ang_z < -self.max_ang: ang_z = -self.max_ang
            twist.linear.x = self.lin_speed
            twist.angular.z = -ang_z  # negative to correct left/right
        else:
            # search mode: rotate slowly and creep forward
            twist.linear.x = self.lin_speed_search
            twist.angular.z = self.search_turn

        self.pub_cmd.publish(twist)

def main():
    rospy.init_node("line_controller")
    PIDController()
    rospy.loginfo("line_controller running...")
    rospy.spin()

if __name__ == "__main__":
    main()
