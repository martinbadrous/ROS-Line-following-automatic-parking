#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Line detection node for TurtleBot3 line following (ROS Melodic / Python2).
Subscribes: sensor_msgs/Image (camera)
Publishes:  std_msgs/Float32 (/line_error)  -> lateral error in pixels relative to image center
           std_msgs/Bool   (/line_visible) -> whether the line is detected
Parameters:
  ~camera_topic        (default: /camera/rgb/image_raw)
  ~output_error_topic  (default: /line_error)
  ~output_visible_topic(default: /line_visible)
  ~hsv_lower           (default: [20,  80,  80])   # tune for your line color
  ~hsv_upper           (default: [35, 255, 255])
  ~blur                (default: 5)
  ~min_area            (default: 1500)
  ~display             (default: false)
"""
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool

class LineDetector(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.pub_err = rospy.Publisher(rospy.get_param("~output_error_topic", "/line_error"), Float32, queue_size=10)
        self.pub_vis = rospy.Publisher(rospy.get_param("~output_visible_topic", "/line_visible"), Bool, queue_size=10)
        self.hsv_lower = np.array(rospy.get_param("~hsv_lower", [20, 80, 80]), dtype=np.uint8)
        self.hsv_upper = np.array(rospy.get_param("~hsv_upper", [35, 255, 255]), dtype=np.uint8)
        self.blur = int(rospy.get_param("~blur", 5))
        self.min_area = int(rospy.get_param("~min_area", 1500))
        self.display = bool(rospy.get_param("~display", False))
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1)

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", str(e))
            return

        h, w = cv_img.shape[:2]
        roi = cv_img[int(h*0.5):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        if self.blur > 0:
            hsv = cv2.GaussianBlur(hsv, (self.blur, self.blur), 0)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        visible = False
        error = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    roi_center = w // 2
                    error = float(cx - roi_center)
                    visible = True
                    if self.display:
                        cv2.drawContours(roi, [c], -1, (0,255,0), 2)
                        cy = int(M["m01"]/max(M["m00"],1))
                        cv2.circle(roi, (cx, cy), 5, (0,0,255), -1)
                        cv2.line(roi, (roi_center,0), (roi_center,roi.shape[0]-1), (255,0,0), 2)

        self.pub_err.publish(Float32(error))
        self.pub_vis.publish(Bool(visible))

        if self.display:
            stacked = np.hstack([roi, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            cv2.imshow("LineDetector ROI | Mask", stacked)
            cv2.waitKey(1)

def main():
    rospy.init_node("line_detector")
    LineDetector()
    rospy.loginfo("line_detector started, waiting for images...")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
