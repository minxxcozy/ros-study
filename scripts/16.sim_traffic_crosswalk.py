#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Float64
from morai_msgs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
from time import *

# 5일차 1:22:00 확률 허프 변환 이전까지 진행된 내용입니다.

class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        
        self.bridge = CvBridge()
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal = 0
        self.signal = 0
        self.cross_flag = 0
        
    def traffic_CB(self, msg):
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN000002":
            signal = self.traffic_msg.trafficLightStatus
            self.signal = signal
            if self.prev_signal != signal:
                self.prev_signal = signal
            first_time = time()
            self.traffic_think()
            second_time = time()
            print(f"diffrent:{second_time - first_time}")
        
    def traffic_think(self):    
        if self.signal == 1:     # stop
            pass
            # print("red")
        elif self.signal == 4:
            pass
            # print("yellow")
        elif self.signal == 16:
            pass
            # print("green")
        elif self.signal == 33:
            pass
            # print("left")
        else:
            pass
        
    
    def cam_CB(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        first_time = time()
        self.cam_lane_detection()
        second_time = time()
        # print(f"diffrent:{second_time - first_time}")
        
    def cam_lane_detection(self):
        y, x = self.img.shape[0:2]
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)
        
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_img = cv2.bitwise_and(self.img, self.img, mask=combined_range)
        
        # Perspective Transformation
        src_point1 = [0, 420]
        src_point2 = [280, 260]
        src_point3 = [x - 280, 260]
        src_point4 = [x, 420]
        src_points = np.float32([src_point1, src_point2, src_point3, src_point4])
        
        dst_point1 = [x // 8, 480]
        dst_point2 = [x // 8, 0]
        dst_point3 = [x // 8 * 7, 0]
        dst_point4 = [x // 8 * 7, 480]
        dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])
        
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(warped_img)
        bin_img[grayed_img > 50] = 255   # 1
        histogram = np.sum(bin_img, axis=0)
        
        left_hist = histogram[0 : x // 2]
        right_hist = histogram[x // 2 :]
        
        left_indices = np.where(left_hist > 20)[0]
        right_indices = np.where(right_hist > 20)[0] + 320
        indices = np.where(histogram > 20)[0]
        
        try:
            if left_indices == 0 and right_indices == 0 :
                center_index = x // 2
                print("no_line")
            elif left_indices != 0 and right_indices == 0:
                center_index = (left_indices[0] + left_indices[-1]) // 2
                print("left_line")
            elif left_indices == 0 and right_indices != 0:
                center_index = (right_indices[0] + right_indices[-1]) // 2
                print("right_line")
            else:
                center_index = (indices[0] + indices[-1]) // 2
                print("both_line")
        except:
            center_index = x // 2
            print("no_line")
        
        # 허프 선 변환
        canny_img = cv2.Canny(bin_img, 2, 2)
        first_time = time()
        lines = cv2.HoughLinesP(bin_img, 1, np.pi/180, 90, 100, 450)
        try:
            for line in lines:
                # 검출된 선 그리기
                x1, y1, x2, y2 = line[0]
                cv2.line(warped_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
                self.cross_flag = self.cross_flag + 1
                print(f"self.cross_flag:{self.cross_flag}")
        except:
            pass
        second_time = time()
        # print(f"diffrent:{second_time - first_time}")
        
        standard_line = x // 2
        degree_per_pixel = 1 / x
        steer = (center_index - standard_line) * degree_per_pixel
        steer = 0.5 + steer    
        # print(f"steer : {steer}")

        self.steer_msg.data = steer
        self.speed_msg.data = 1000
        
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)
        
        # cv2.imshow("canny_img", canny_img)
        cv2.imshow("img", self.img)
        cv2.imshow("warped_img", warped_img)
        cv2.waitKey(1)  
            
    
def main():
    try:
        traffic_control = Traffic_control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
if __name__ == "__main__":
    main()