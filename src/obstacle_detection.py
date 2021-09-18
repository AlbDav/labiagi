#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from collision_avoidance.srv import ForceResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class obstacleDetection:
    def __init__(self):
        self.sub = rospy.Subscriber('/base_scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('marker', Marker, queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.ns = 'robot'
        self.marker.id = 0
        self.marker.type = 0
        self.marker.action = 0
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        q = quaternion_from_euler(0, 0, 3.14)
        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]
        self.marker.scale.x = 4
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.lifetime = rospy.Duration(0)

        while not rospy.is_shutdown():
            self.pub.publish(self.marker)
            time.sleep(1)
        # force = ForceResponse(3,2)
        # print(force.magnitude)
        # print(force.angle)

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