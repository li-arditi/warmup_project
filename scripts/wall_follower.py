#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import radians

distance = 0.4

class FollowWall(object):
    def __init__(self):
        rospy.init_node("follow_wall")
        # rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        # self.task = "find_wall"

    def process_scan(self, data):
        # while not rospy.is_shutdown():
        # if data.ranges[0] >= distance:
        #     self.twist.linear.x = 0.2
        # else:
        #     self.twist.linear.x = 0 
        # self.twist_pub.publish(self.twist)
        # self.twist.angular.z = radians(45)
        
        dist_closest_wall = float('inf')
        ang_closest_wall = 0
        for a in range(0, len(data.ranges)):
            if data.ranges[a] < dist_closest_wall:
                dist_closest_wall = data.ranges[a]
                ang_closest_wall = a
                # rospy.loginfo(a)
            else:
                continue
        rospy.loginfo(dist_closest_wall)
        rospy.loginfo(ang_closest_wall)
        
        if dist_closest_wall == 'inf':
            rospy.loginfo("if dist is inf")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
        else:
            rospy.loginfo("rotating")
            if ang_closest_wall > 180:
                self.twist.angular.z = -radians((360 - ang_closest_wall)/5)
            else:
                self.twist.angular.z = radians(ang_closest_wall/5)
            self.twist_pub.publish(self.twist)
            rospy.loginfo(self.twist)
        rospy.sleep(5.0)
        self.twist.angular.z = 0
        rospy.loginfo(self.twist)
        self.twist_pub.publish(self.twist)
        
        rospy.loginfo("did we get here")
        if data.ranges[0] >= distance:
            self.twist.linear.x = 0.2
        else:
            self.twist.linear.x = 0 
        self.twist_pub.publish(self.twist)


        #     while data.ranges[0] >= distance:
        #         self.twist.linear.x = 0.2
        #         self.twist_pub.publish(self.twist)

        #     self.twist.linear.x = 0 
        #     self.twist_pub.publish(self.twist)




    
    # def shutdown(self):
    #     self.twist_pub.publish(Twist())
    #     rospy.sleep(1)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FollowWall()
    node.run()