#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import radians

distance = 0.4
FIND_WALL_STATE = 0
FOLLOWER_STATE = 1

class FollowWall(object):
    def __init__(self):
        rospy.init_node("follow_wall")
        # rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.state = FIND_WALL_STATE
    
    def find_closest(self, ranges):
        dist_closest_wall = float('inf')
        ang_closest_wall = 0
        for a in range(0, len(ranges)):
            if ranges[a] < dist_closest_wall:
                dist_closest_wall = ranges[a]
                ang_closest_wall = a
                # rospy.loginfo(a)

        return (dist_closest_wall, ang_closest_wall)

    def find_wall(self, ranges):
        
        (dist_closest_wall, ang_closest_wall) = self.find_closest(ranges)

        if dist_closest_wall == 'inf':
            # no walls around so just stay where you are
            rospy.loginfo("if dist is inf")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            
        elif ang_closest_wall not in [0,1,2,3,359,358,357]:
            # not facing wall, so need to turn to it
            if ang_closest_wall > 180:
                error= radians(360-ang_closest_wall)
            else:
                error = radians(0 - ang_closest_wall)
            self.twist.angular.z = -error/10
        
        elif ang_closest_wall in [0,1,2,3,359,358,357] and dist_closest_wall > distance:
            # we are facing the wall and farther than distance away
            self.twist.angular.z = 0

        else:
            # we are facing the wall but closer than distance away so just turn 90

        self.twist_pub.publish(self.twist)
            # there is a (closest) wall so rotate to it...
        #     rospy.loginfo("rotating")
        #     if ang_closest_wall > 180:
        #         self.twist.angular.z = -radians((360 - ang_closest_wall)/15)
        #     else:
        #         self.twist.angular.z = radians(ang_closest_wall/15)
        #     self.twist_pub.publish(self.twist)
        #     rospy.loginfo(self.twist)
        
        # rospy.sleep(15.0)
        # self.twist.angular.z = 0
        # rospy.loginfo(self.twist)
        # self.twist_pub.publish(self.twist)

    
    def follow_wall(self, data):
        # self.twist.linear.x = 0.2
        # self.twist_pub.publish(self.twist)
        pass

    def handle_corner(self, data):
        pass


    def process_scan(self, data):
        if self.state == FIND_WALL_STATE:
            self.find_wall(data.ranges)
            
        
        
        
        # if data.ranges[0] >= distance:
        #     self.twist.linear.x = 0.2
        # else:
        #     self.twist.linear.x = 0 
        # self.twist_pub.publish(self.twist)


        #     while data.ranges[0] >= distance:
        #         self.twist.linear.x = 0.2
        #         self.twist_pub.publish(self.twist)

        #     self.twist.linear.x = 0 
        #     self.twist_pub.publish(self.twist)



    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FollowWall()
    node.run()