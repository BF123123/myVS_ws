#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisualizerNode:
    def __init__(self):
        rospy.init_node('vs_visualizer', anonymous=True)
        self.bridge = CvBridge()

        # 定义窗口名称
        self.win_curr = "1. Current View"
        self.win_ref  = "2. Desired View (Target)"
        self.win_err  = "3. Error Heatmap"

        # 创建窗口并指定位置 (避免重叠)
        # 注意：不同屏幕分辨率可能需要调整位置 (x, y)
        cv2.namedWindow(self.win_curr, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.win_curr, 50, 100)
        cv2.resizeWindow(self.win_curr, 400, 300)

        cv2.namedWindow(self.win_ref, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.win_ref, 500, 100)
        cv2.resizeWindow(self.win_ref, 400, 300)

        cv2.namedWindow(self.win_err, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.win_err, 950, 100)
        cv2.resizeWindow(self.win_err, 400, 300)

        # 订阅图像话题 (请确保这些话题名称与 Part 2 中发布的一致)
        rospy.Subscriber("/vs/image_processed", Image, self.cb_current)
        rospy.Subscriber("/vs/image_desired_gray", Image, self.cb_desired)
        rospy.Subscriber("/vs/image_error_control", Image, self.cb_error)

        rospy.loginfo("Visualizer Dashboard Started.")
        
        # 循环刷新窗口
        self.spin()

    def cb_current(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv2.imshow(self.win_curr, cv_img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            pass

    def cb_desired(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv2.imshow(self.win_ref, cv_img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            pass

    def cb_error(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "mono8")
            # 可以在这里加个伪彩色处理，让误差更明显
            color_err = cv2.applyColorMap(cv_img, cv2.COLORMAP_JET)
            cv2.imshow(self.win_err, color_err)
            cv2.waitKey(1)
        except CvBridgeError as e:
            pass

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            cv2.waitKey(10)
            rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        VisualizerNode()
    except rospy.ROSInterruptException:
        pass