#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from math import radians




class DriveInSquare(object):
    def __init__(self):
        rospy.init_node('drive_in_square')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def run(self):

        r = rospy.Rate(5)
        count = 0

        move_cmd = Twist()
        move_cmd.linear.x = 0.2

        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(45)

        while not rospy.is_shutdown() and count < 4:
            for x in range(0,20):
                self.twist_pub.publish(move_cmd)
                r.sleep()
            for x in range(0,10):
                self.twist_pub.publish(turn_cmd)
                r.sleep()
            count += 1
    
    def shutdown(self):
        self.twist_pub.publish(Twist())
        rospy.sleep(1)
        

if __name__ == '__main__':
    node = DriveInSquare()
    node.run()

    