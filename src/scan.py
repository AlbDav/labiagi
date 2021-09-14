#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import math

def callback(msg):
    print(msg)
    for i, val in enumerate(msg.ranges, start=0):
        angle = msg.angle_min + (i * msg.angle_increment)
        x = val * math.cos(angle)
        y = val * math.sin(angle)
        print(x, y, angle, val)
        if(val > 10):
            continue
        # print(i, val)
    time.sleep(120)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/base_scan', LaserScan, callback)
rospy.spin()