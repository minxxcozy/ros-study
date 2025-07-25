#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Turtle_sub:
    def __init__(self):
        rospy.init_node("turtle_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.bridge = CvBridge()

    def cam_CB(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("cv_img", cv_img)
        cv2.waitKey(1)
        # print("cam")

def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()