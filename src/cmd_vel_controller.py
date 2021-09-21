#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from random import random
from time import sleep
from collision_avoidance.srv import Force, ForceResponse
import math

class cmdVelController:
    def __init__(self):
        self.pub = rospy.Subscriber('/cmd_vel_input', Twist, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    def callback(self, msg):
        if msg.linear.x == 0:
            self.pub.publish(msg)
            return
        rospy.wait_for_service('force_service')
        try:
            force_service = rospy.ServiceProxy('force_service', Force)
            force = force_service()
            vel_msg = Twist()
            x_linear = msg.linear.x + force.magnitude
            vel_msg.linear.x = x_linear if x_linear < 0.75 else 0.75
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = msg.angular.z + force.angle
            print(vel_msg)
            self.pub.publish(vel_msg)
        except rospy.ServiceException as e:
            print('Service call failed: %s' %e)


def main():
    rospy.init_node('cmd_vel_controller', anonymous=True)
    cmd = cmdVelController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()