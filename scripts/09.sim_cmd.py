#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from turtlesim.msg import Pose, Color
from std_msgs.msg import Float64

class Turtle_sub:
    def __init__(self):
        rospy.init_node("sim_cmd_node")
        self.pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.cmd_msg = Float64()
        self.rate = rospy.Rate(100)
    
    def func(self):
        self.speed += 1
        if speed >= 2400:
            speed = 2400 
        self.cmd_msg.data = self.speed
        self.pub.publish(self.cmd_msg)
        print(f"speed : {self.cmd_msg.data}")
        self.rate.sleep()
  
    
def main():
    try:
        turtle_sub = Turtle_sub()
        while not rospy.is_shutdown():
            turtle_sub.func()
    except rospy.ROSInterrupException:
        pass

if __name__ == "__main__":
    main()