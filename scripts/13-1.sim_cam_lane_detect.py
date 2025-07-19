#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np  # 영상 구조와 표현 방법

# 색상, 이미지, 도형 실습 ('''''' 지워서 하나씩 실습 진행)

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
      
    # 색상 (1)
    def cam_CB(self, msg):
        grayscale = np.array([
                    [255, 0, 128, 0, 255],
                    [0, 255, 128, 255, 0],
                    [255, 0, 128, 0, 255],
                    [0, 255, 128, 255, 0]], np.uint8)
        cv2.namedWindow("grayscale", cv2.WINDOW_NORMAL)
        cv2.imshow("grayscale", grayscale)
        cv2.waitKey(1)
        
        # self.image_msg = msg
        # cv_img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
        # cv2.imshow("cv_img", cv_img)
        # cv2.waitKey(1)
        # print("cam")
    
    '''
    # 색상 (2)
    def cam_CB(self, msg):
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]
        costom = [255, 255, 255]
        color = costom
        image = np.array([
                    [color, color, color, color, color],
                    [color, color, color, color, color],
                    [color, color, color, color, color],
                    [color, color, color, color, color]], np.uint8)
        y, x = image.shape[0:2]
        print(y, x)
        print(image.shape)
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", image)
        cv2.waitKey(1)    
    '''
    
    '''
    # 이미지 입출력
    def cam_CB(self, msg):
        img = cv2.imread("image_file.jpg", cv2.IMREAD_COLOR)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        key = cv2.waitKey(0)
        cv2.imwrite("image_file_gray.jpg", img)
        
        # self.image_msg = msg
        # cv_img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
        # cv2.imshow("cv_img", cv_img)
        # cv2.waitKey(1)
        # print("cam")      
    '''
    
    '''
    # 도형 그리기
    def cam_CB(self, msg):
        zero = np.zeros([480, 640, 3], np.uint8)
        cv2.line(zero, [240, 320], [240, 320], [0, 0, 255], 3)
        cv2.rectangle(zero, [120, 160], [280, 320], [255, 0, 0], -1)
        y, x, _ = zero.shape
        
        cv2.imshow("zero", zero)
        cv2.waitKey(0)
    '''
    
        
def main():
    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()