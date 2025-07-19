#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from turtlesim.msg import Pose, Color
from std_msgs.msg import Float64

class Turtle_sub:
    def __init__(self):
        rospy.init_node("sim_cmd_node")
        self.pub = rospy.Publisher("/commands/server/position", Float64, queue_size=1)
        self.cmd_msg = Float64()
        self.rate = rospy.Rate(100)
        self.steer = 0 # value: 0 ~ 1 -> degree: -19.5 ~ 19.5 
    
    def func(self):
        # self.steer += 1
        # self.steer = self.steer % 19.5
        # self.cmd_msg.data = ((self.steer / 19.5) + 1) / 2
        self.steer += 0.0   # 0.1 등 증가시키기
        if self.steer >= 1:
            self.steer = 1
        self.cmd_msg.data = self.steer
        self.pub.publish(self.cmd_msg)
        print(f"steer : {self.cmd_msg.data}")
        self.rate.sleep()
  
    
def main():
    try:
        turtle_sub = Turtle_sub()
        while not rospy.is_shutdown():
            turtle_sub.func()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()