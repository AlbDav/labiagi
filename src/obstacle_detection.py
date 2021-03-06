#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from collision_avoidance.srv import Force, ForceResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class obstacleDetection:
    def __init__(self):
        self.sub = rospy.Subscriber('/base_scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('marker', Marker, queue_size=10)
        self.srv = rospy.Service('force_service', Force, self.force_service)
        self.force = ForceResponse(0, 0)

    def callback(self, msg):
        self.force = ForceResponse(0, 0)
        for i, val in enumerate(msg.ranges, start=0):
            if val >= 1.25:
                continue
            magnitude = 1 / val
            temp_angle = msg.angle_min + (i * msg.angle_increment)
            x = magnitude * math.cos(temp_angle)
            y = magnitude * math.sin(temp_angle)
            angle = math.atan2(-y, -x)
            force = ForceResponse(magnitude, angle)
            self.set_net_force(force)
        self.show_force()
    
    def set_net_force(self, force):
        x1 = self.force.magnitude * math.cos(self.force.angle)
        y1 = self.force.magnitude * math.sin(self.force.angle)
        x2 = force.magnitude * math.cos(force.angle)
        y2 = force.magnitude * math.sin(force.angle)
        x_total = x1 + x2
        y_total = y1 + y2
        total_magnitude = math.sqrt(x_total ** 2 +  y_total ** 2)
        total_angle = math.atan2(y_total, x_total)
        self.force = ForceResponse(total_magnitude, total_angle)

    def show_force(self):
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
        q = quaternion_from_euler(0, 0, self.force.angle)
        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]
        self.marker.scale.x = 0.1 * self.force.magnitude if self.force.magnitude > 0.05 else 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.lifetime = rospy.Duration(0)

        self.pub.publish(self.marker)

    def force_service(self, request):
        return self.force

def main():
    rospy.init_node('obstacle_detection', anonymous=True)
    od = obstacleDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()