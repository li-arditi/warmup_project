#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import radians

goal_dist = 0.4
FIND_WALL_STATE = 0
FOLLOWER_STATE = 1
TURN_STATE = 2
CORNER_STATE = 3

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
        # rospy.loginfo(dist_closest_wall)
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
            self.twist.angular.z = -error/5
        
        elif ang_closest_wall in [0,1,2,3,359,358,357] and dist_closest_wall > goal_dist:
            # we are facing the wall and farther than goal_dist away
            self.twist.angular.z = 0
            self.twist.linear.x = 0.15

        else:
            # we are facing the wall and closer than or at goal_dist away
            # so turn to be parallel with wall
            self.twist.linear.x = 0
            self.state = TURN_STATE

        self.twist_pub.publish(self.twist)
            
        
       
    def turn(self, ranges):
        rospy.loginfo("we are about to turn")
        (dist_closest_wall, ang_closest_wall) = self.find_closest(ranges)
        # self.twist.linear.x = 0
        # self.twist_pub.publish(self.twist)
        rospy.loginfo(ang_closest_wall)
        if ang_closest_wall not in range(87,93):
            # not at 90 deg 
        #     error = radians(90 - ang_closest_wall)
            self.twist.angular.z = -0.1
        else:
            self.twist.angular.z = 0
            self.state = FOLLOWER_STATE
        self.twist_pub.publish(self.twist)


    
    def follow_wall(self, ranges):

        rospy.loginfo("we are about to follow wall")
        self.twist.linear.x = 0.2
        self.twist_pub.publish(self.twist)
        (dist_closest_wall, ang_closest_wall) = self.find_closest(ranges)
        rospy.loginfo(dist_closest_wall)
        if ranges[0] > goal_dist and dist_closest_wall != goal_dist:
            error = goal_dist - dist_closest_wall # too close gives pos
            self.twist.angular.z = -error/5
        elif ranges[0] <= goal_dist:
            rospy.loginfo("we detected a wall")
            self.twist = Twist()
            # self.twist_pub.angular.z = -0.1
            self.state = FIND_WALL_STATE
        self.twist_pub.publish(self.twist)

            
        


    def process_scan(self, data):
        if self.state == FIND_WALL_STATE:
            self.find_wall(data.ranges)
        elif self.state == TURN_STATE:
            self.turn(data.ranges)
        elif self.state == FOLLOWER_STATE:
            self.follow_wall(data.ranges)

        
        
        # if data.ranges[0] >= goal_dist:
        #     self.twist.linear.x = 0.2
        # else:
        #     self.twist.linear.x = 0 
        # self.twist_pub.publish(self.twist)


        #     while data.ranges[0] >= goal_dist:
        #         self.twist.linear.x = 0.2
        #         self.twist_pub.publish(self.twist)

        #     self.twist.linear.x = 0 
        #     self.twist_pub.publish(self.twist)



    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FollowWall()
    node.run()