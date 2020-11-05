#!usr/bin/env python

import rospy
import tf
import random


from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import LaserScan

from math import radians, pi as PI, sqrt

THREE_METERS = 3

RANDOM_WALK = 0
AVOID = 1
WALL = 2

OBSTACLE = False

SPEED = 0.2
STATIONARY = 0


class Nav():

    def __init__(self):
        self.velocity = Twist()
        #velocity includes forward speed and direction
        self.position.x = None
        self.position.y = None
        self.behaviour = RANDOM_WALK

    def move_turn(self):
        """
        Moves in an already cleared direction and turns in a random direction
        """
        self.velocity.linear.x = SPEED
        if sqrt((self.position.x - start_point.x) ** 2 +
                (self.position.y - start_point.y) ** 2) < THREE_METERS:
            self.velocity.publish(self.velocity)
        else:
            self.velocity.publish(Twist())
            spin()
            start_point.x = self.position.x
            start_point.y = self.position.y

    def scan(self, scandata)
        """
        sudo code while i cant run this
        """
        OBSTACLE = scandata.ranges
        """
        if there are obstacles
            if there is a wall
                run wall
            else
                run spin
        else
            run move_turn
        """

    def spin(self)
        self.velocity.theta = random.uniform(0, PI * 2)
        self.velocity.publish(self.velocity.theta)

    
    def odom_callback(self, odom):
        quart = [odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(quart)

        self.velocity.theta = yaw
        self.position.x = odom.pose.pose.position.x
        self.position.y = odom.pose.pose.position.y

if __name__ == "__main__":
    try:
        RobotControl().start()
    except rospy.ROSInterruptException:
        pass