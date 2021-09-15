#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from classes.Force import Force

class obstacleDetection:
    def __init__(self):
        self.sub = rospy.Subscriber('/base_scan', LaserScan, self.callback)
        force = Force(12, 4)
        print(force.magnitude)
        print(force.angle)

    def callback(self, msg):
        # print(msg)
        for i, val in enumerate(msg.ranges, start=0):
            angle = msg.angle_min + (i * msg.angle_increment)
            x = val * math.cos(angle)
            y = val * math.sin(angle)
            # print(x, y, angle, val)
            if(val > 10):
                continue
            # print(i, val)
        time.sleep(120)

def main():
    rospy.init_node('obstacle_detection', anonymous=True)
    od = obstacleDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()