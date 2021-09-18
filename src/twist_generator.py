#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from random import random
from time import sleep

class twistGenerator:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move()
    
    def move(self):
        vel_msg = Twist()
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0;
        vel_msg.angular.z = -5 + random() * 10;
        #while not rospy.is_shutdown():
        #    vel_msg.linear.x = random() * 100;
        #    self.pub.publish(vel_msg)
        #    sleep(1)


def main():
    rospy.init_node('twist_generator', anonymous=True)
    twist = twistGenerator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
