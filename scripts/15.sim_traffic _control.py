#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Float64
from morai_msgs.msg import GetTrafficLightStatus
from cv_bridge import CvBridge
import numpy as np

class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.prev_signal = 0
        
    def traffic_CB(self, msg):
        # print(msg)
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN000002":
            signal = self.traffic_msg.trafficLightStatus
            if self.prev_signal != signal:
                self.prev_signal = signal
                self.traffic_flag = 0
        
            self.steer_msg = 0.5
            self.traffic_flag += 1
        
            if signal == 1:     # stop
                self.speed_msg.data = 0
                print(f"red: {self.traffic_flag}")
            elif signal == 4:
                self.speed_msg.data = 1000
                print(f"yellow: {self.traffic_flag}")
            elif signal == 16:
                self.speed_msg.data = 3000
                print(f"green: {self.traffic_flag}")
            elif signal == 33:
                self.speed_msg.data = 1000   
                self.steer_msg.data = 0.4
                print(f"left: {self.traffic_flag}")
            else:
                print(f"{signal}:{self.traffic_flag}")
                self.speed_msg.data = 1000
                self.steer_msg.data = 0.5
            
            self.speed_pub.publish(self.speed_msg)
            self.steer_pub.publish(self.steer_msg)
        
        else:
            pass
            
    
def main():
    try:
        traffic_control = Traffic_control()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
if __name__ == "__main__":
    main()