#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math

goal_dist = 0.4
FIND_WALL_STATE = 0
FOLLOWER_STATE = 1
TURN_STATE = 2
CORNER_STATE = 3

class FollowWall(object):
    def __init__(self):
        rospy.init_node("follow_wall") # initialize node

        #subscribe to /scan and publish to /cmd_vel
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.state = FIND_WALL_STATE # set initial state

    
    def find_closest(self, ranges, within = (0, 360)):
        """
        This functions finds the closest object (wall) to the robot within the 
        given angular range
            ranges: list from LaserScan message of distances at different degrees
            within: optional argument to specify range to find closest (not used in
            this implementation but could potentially use in different implementation)
        
        Returns: tuple of distance to closest object and angle of that closest object
        """

        (min, max) = within
        dist_closest_wall = float('inf')
        ang_closest_wall = 0
        for a in range(min, max):
            if ranges[a] < dist_closest_wall:
                dist_closest_wall = ranges[a]
                ang_closest_wall = a
                # rospy.loginfo(a)

        return (dist_closest_wall, ang_closest_wall)


    def find_wall(self, ranges):
        """
        Called when self.state = FIND_WALL_STATE; finds the closest wall, gets
        the goal distance away from it and rotates to be parallel to the wall
            ranges: distance data from LaserScan message
        """

        (dist_closest_wall, ang_closest_wall) = self.find_closest(ranges)
        if dist_closest_wall == 'inf':
            # no walls around so just stay where you are
            self.twist.linear.x = 0
            self.twist.angular.z = 0

        elif ang_closest_wall not in [0,1,2,3,359,358,357]:
            # not facing wall, so need to turn to it
            if ang_closest_wall > 180:
                # closest is to right of robot
                error= math.radians(360-ang_closest_wall)
            else:
                error = math.radians(0 - ang_closest_wall)
            self.twist.angular.z = -error/5
        
        elif ang_closest_wall in [0,1,2,3,359,358,357] and dist_closest_wall > goal_dist:
            # we are facing the wall and farther than goal_dist away
            self.twist.angular.z = 0
            error = dist_closest_wall - goal_dist
            self.twist.linear.x = error/3

        else:
            # we are facing the wall and at goal_dist away so turn to be parallel
            self.twist.linear.x = 0
            self.state = TURN_STATE

        self.twist_pub.publish(self.twist)

       
    def turn(self, ranges):
        """
        Called when self.state = TURN_STATE; rotates to be parallel to the wall
            ranges: distance data from LaserScan message
        """

        (dist_closest_wall, ang_closest_wall) = self.find_closest(ranges)
        if ang_closest_wall not in range(87,93):
            # not parallel to wall yet 
            self.twist.angular.z = -0.1
        else:
            self.twist.angular.z = 0
            self.state = FOLLOWER_STATE
        self.twist_pub.publish(self.twist)

    
    def follow_wall(self, ranges):
        """
        Called when self.state = FOLLOW_WALL_STATE; follows the wall staying apprx.
        goal distance away; if it detects a wall change to CORNER_STATE
            ranges: distance data from LaserScan message
        """

        self.twist.linear.x = 0.2
        self.twist_pub.publish(self.twist)
       
        if ranges[0] <= 1.2:
            # we detect a wall in front of us so switch to CORNER_STATE
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.1
            self.state = CORNER_STATE
        else:
            # no corner detected so continue straight
            error = goal_dist - ranges[90] # too close gives pos
            self.twist.angular.z = -error/3
        
        self.twist_pub.publish(self.twist)


    def handle_corner(self, ranges):
        """
        Called when self.state = CORNER_STATE; rotates until robot is parallel
        to the new wall it had detected
            ranges: distance data from LaserScan message
        """
        # linear.x = 0.1 set from follow_wall() 
        
        (dist_closest_new, ang_closest_new) = self.find_closest(ranges)
        if ang_closest_new in range(85,95):
            # not parallel to wall yet 
            self.twist.angular.z = 0
            self.state = FOLLOWER_STATE
        else:
            self.twist.angular.z = -0.2
        
        self.twist_pub.publish(self.twist)


    def process_scan(self, data):
        """
        Callback function for Subscriber to /scan/LaserScan; calls appropriate functions
            data: data from LaserScan message
        """

        if self.state == FIND_WALL_STATE:
            self.find_wall(data.ranges)
        elif self.state == TURN_STATE:
            self.turn(data.ranges)
        elif self.state == FOLLOWER_STATE:
            self.follow_wall(data.ranges)
        elif self.state == CORNER_STATE:
            self.handle_corner(data.ranges)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FollowWall()
    node.run()