#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from turtlesim.msg import Pose, Color
from geometry_msgs.msg import Twist

class Turtle_sub:
    def __init__(self):
        rospy.init_node("turtle_pub_node")
        rospy.Subscriber("/turtle1/pose", Pose, self.turtle_pose_CB)
        rospy.Subscriber("/turtle/color_sensor", Color, self.turtle_color_CB)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.pose_msg = Pose()
        self.color_msg = Color()
        self.cmd_msg = Twist()
        self.rate = rospy.Rate(10)
    
    def turtle_pose_CB(self, msg):
        print("----pose----")
        self.pose_msg = msg
        print(self.pose_msg)
        self.cmd_msg.linear.x = 1
        self.pub.publish(self.cmd_msg)
        self.rate.sleep()
    
    def turtle_color_CB(self, msg):
        print("----color----")
        self.color_msg = msg
        print(self.color_msg)
        self.cmd_msg.linear.x = 0.5
        self.pub.publish(self.cmd_msg)
        self.rate.sleep()
    
def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()