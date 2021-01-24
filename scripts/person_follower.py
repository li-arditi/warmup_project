#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math

distance = 0.4

class FollowPerson(object):
    def __init__(self):
        rospy.init_node("follow_person")
        # rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

    def find_closest_obj(self, ranges):
        dist_closest_obj = float('inf')
        ang_closest_obj = 0
        for a in range(0, len(ranges)):
            if ranges[a] < dist_closest_obj:
                dist_closest_obj = ranges[a]
                ang_closest_obj = a
        return (dist_closest_obj, ang_closest_obj)

    def ang_error(self, angle):
        if angle > 180:
            error_ang = math.radians(360-angle)
        else:
            error_ang = math.radians(0 - angle)
        return error_ang

    def process_scan(self, data):
        (dist_closest_obj, ang_closest_obj) = self.find_closest_obj(data.ranges)
        error_ang = self.ang_error(ang_closest_obj)

        if dist_closest_obj == float('inf'):
            # nothing within scanning range so just stay where you are
            rospy.loginfo("if dist is inf")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        
        
        elif ang_closest_obj in [0,1,2,3,359,358,357]:
            self.twist.angular.z = 0

        elif ang_closest_obj not in range(0,45) and ang_closest_obj not in range(315, 360):
            self.twist.linear.x = 0.15
            self.twist.angular.z = -error_ang/2

        else:
            error_dist = abs(dist_closest_obj - distance)
            
            self.twist.linear.x = error_dist/3
            self.twist.angular.z = -error_ang/2
        
        if dist_closest_obj <= distance:
            self.twist.linear.x = 0

        self.twist_pub.publish(self.twist)

    
            


    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FollowPerson()
    node.run()