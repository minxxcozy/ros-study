#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import os
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from math import *


class Turtle_sub:
    def __init__(self):
        rospy.init_node("sim_e_stop")
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.image_msg = LaserScan()
        self.speed_msg = Float64()
        self.steer_msg = Float64()

        
    def lidar_CB(self, msg):
        os.system("clear")
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180/pi
        degree_max = self.scan_msg.angle_max * 180/pi
        degree_angle_increment = self.scan_msg.angle_increment * 180/pi
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)]
        
        degree_array = np.array(degrees)

        obstacle_degrees = []
        obstacle_index = []
        # index 180 (90), 542 (-90)
        
        for index, value in enumerate(self.scan_msg.ranges):
            if abs(degrees[index]) < 90 and 0 < value < 1.5:   # 각도 값 & 거리 값 함께 고려
                obstacle_degrees.append(degrees[index])
                obstacle_index.append(index)
            else:
                pass
        
        try:
            right_space = obstacle_index[0] - 180
            left_space = 542 - obstacle_index[-1]
            
            if right_space > left_space:
                print(f"right_space: {obstacle_index[0]}")
                left_index_avg = (obstacle_degrees[obstacle_index[0]] - 90) / 2
                print(f"right_degree_avg: {left_index_avg}")
                degree_avg = left_index_avg 
            else:
                print(f"left: {obstacle_index[-1]}")
                right_index_avg = (obstacle_degrees[obstacle_index[-1]] + 90) / 2
                print(f"left_degree_avg: {right_index_avg}")
                degree_avg = right_index_avg
            
            steer = ((-degree_avg / 90) + 0.5)
        
        except:
            degree_avg = 0
            steer = 0.5
            # 0 > -19.5 | 0.5 > 0 | 1 > 19.5 (degree)
        
        print(steer)
        
        self.speed_msg.data = 1000
        self.steer_msg.data = steer
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)
                  

def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()