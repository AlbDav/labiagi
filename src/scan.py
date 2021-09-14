#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import math

class obstacleDetection:
    def __init__(self):
        self.sub = rospy.Subscriber('/base_scan', LaserScan, self.callback)

    def callback(self, msg):
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

def main():
    od = obstacleDetection()
    rospy.init_node('obstacle_avoidance', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()