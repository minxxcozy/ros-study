#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

# 4일차 4:20:00 부터는 값 조정하면서 작업하시면 됩니다!
# 추가로 indices 조건문도 추가하셔야 해요.

class Lane_sub:
    def __init__(self):
        rospy.init_node("land_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
        self.speed_msg = Float64()
        self.steer_msg = Float64()

    '''
    def cam_CB(self, msg):
        # img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow("img", img)
        # cv2.waitKey(1)
        color = [240, 10, 10]
        color1 = [255, 0, 0]
        img = np.array([
                    [color, color, color, color, color],
                    [color, color, color, color, color],
                    [color, color, color, color, color],
                    [color, color, color, color, color]], np.uint8)
        
        img1 = np.array([
                    [color1, color1, color1, color1, color1],
                    [color1, color1, color1, color1, color1],
                    [color1, color1, color1, color1, color1],
                    [color1, color1, color1, color1, color1]], np.uint8)
        
        cv2.nameWindow("img", cv2.WINDOW_NORMAL)
        cv2.nameWindow("img1", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("img1", img1)
        cv2.waitKey(1)  
    '''
    
    '''
    def cam_CB(self, msg):
        img = cv2.imread("image_file.jpg")
        b, g, r =cv2.split(img)
        # img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_bgr = cv2.merge(b, g, r)
        cv2.nameWindow("img", cv2.WINDOW_NORMAL)
        cv2.nameWindow("img_bgr", cv2.WINDOW_NORMAL)
        cv2.nameWindow("b", cv2.WINDOW_NORMAL)
        cv2.nameWindow("g", cv2.WINDOW_NORMAL)
        cv2.nameWindow("r", cv2.WINDOW_NORMAL)
        # cv2.nameWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("img_bgr", img_bgr)
        cv2.imshow("b", b)
        cv2.imshow("g", g)
        cv2.imshow("r", r)
        # cv2.imshow("img", img_hsv)
        cv2.waitKey(0)     
    '''   
    
    '''
    # h, s, v
    def cam_CB(self, msg):
        print("asd")
        img = cv2.bridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)
        cv2.imshow("h", h)
        cv2.imshow("s", s)
        cv2.imshow("v", v)
        lower = [15, 0, 0]
        upper = [45, 255, 255]
        img_range = cv2.inRange(img_hsv, lower, upper)
        cv2.imshow("img", img)
        cv2.imshow("img_hsv", img_range)
        # cv2.imshow("img_hsv", img_hsv)
        cv2.waitKey(1)   
    '''
    
    '''
    def cam_CB(self, msg):
        img = cv2.bridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)
        
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        
        filtered_img = cv2.bitwise_and(img, img, mask=combined_range)
        
        cv2.imshow("img", img)
        # cv2.imshow("img_range", yellow_range)
        # cv2.imshow("img_hsv", white_range)
        cv2.imshow("combined_range", combined_range)
        cv2.imshow("filtered_img", filtered_img)
        cv2.waitKey(1)
    '''       
        
    def cam_CB(self, msg):
        img = cv2.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[0:2]
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)
        
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        
        filtered_img = cv2.bitwise_and(img, img, mask=combined_range)
        
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
        warped_img = cv2.warpPerspective(filtered_img, matrix, [x, y])
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BRG2GRAY)
        bin_img = np.zeros_like(warped_img)
        bin_img[grayed_img > 50] = 255   # 1
        
        histogram = np.sum(bin_img, axis=0)
        # print(histogram)
        # print(histogram[histogram != 0])
        # print(np.sum(bin_img, axis=0))
        left_hist = histogram[0 : x//2]
        rigth_hist = histogram[x // 2 :]
        
        indices = np.where(histogram > 20)[0]
        # print(indices)
        
        try:
            center_index = (indices[0] + indices[-1]) // 2
        except:
            center_index = x // 2
        print(center_index)
        
        standard_line = x // 2
        degree_per_pixel = 1 / x
        # steer = - (center_index - standard_line) * 0.01
        # steer = ((center_index - standard_line) * 0.005 / 3.2) / 2
        steer = (center_index - standard_line) * degree_per_pixel
        steer = 0.5 + steer 
        # print(0.5 + steer)    
        # print(f"steer : {steer}")

        self.steer_msg.data = steer
        self.speed_msg.data = 1000
        
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)
        
        cv2.imshow("img", img)
        # cv2.imshow("bin_img", bin_img)    
        
        # cv2.imshow("img_range", yellow_range)
        # cv2.imshow("img_hsv", white_range)
        
        # cv2.imshow("combined_range", combined_range)
        
        # cv2.imshow("filtered_img", filtered_img)
        cv2.imshow("warped_img", warped_img)
        cv2.waitKey(1)  
        
def main():
    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()