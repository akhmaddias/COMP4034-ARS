#!usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import radians, sqrt
import random

WALKING_DISTANCE = 3
MIN_DISTANCE = 0.5


class Behaviour():
    def __init__(self):
        # self.behaviour = input("Enter behaviour mode - \n\t"
        #                        + "'0' for obstacle avoidance"
        #                        + " + random walk \n\t"
        #                        + "'1' for right hand wall following"
        #                        + " + random walk\nMode:")
        rospy.init_node('behaviour')

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.rate = rospy.Rate(10)

        self.move_forward = Twist()
        self.move_forward.linear.x = 0.2

        self.turn_left = Twist()
        self.turn_left.linear.x = 0
        self.turn_left.angular.z = radians(30)

        self.turn_right = Twist()
        self.turn_right.linear.x = 0
        self.turn_right.angular.z = radians(-30)

        self.slide_right = Twist()
        self.slide_right.linear.x = 0.1
        self.slide_right.angular.z = radians(-10)

        self.current_pose_theta = None
        self.current_pose_x = None
        self.current_pose_y = None

        self.last_pose_theta = None
        self.last_pose_x = None
        self.last_pose_y = None

        self.continue_right = False
        self.continue_left = False
        self.save_position = True
        self.is_wall_found = False

        # Scan windows
        self.front = None
        self.right_front = None
        self.right_back = None
        self.right = None
        self.back = None
        self.left_back = None
        self.left_front = None
        self.left = None

    def start(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,
                                          self.scan_callback)
        rospy.spin()

    def publish_command(self, msg):
        self.cmd_vel_pub.publish(msg)

    def keep_left(self):
        self.continue_right = False
        self.continue_left = True
        self.publish_command(self.turn_left)

    def keep_right(self):
        self.continue_right = True
        self.continue_left = False
        self.publish_command(self.turn_right)

    def right_hand_wall_follow(self):
        rospy.loginfo("Right hand wall following")
        self.is_wall_found = True

        if min(self.front) <= MIN_DISTANCE:
            self.obstacle_avoidance()
        elif min(self.right) <= MIN_DISTANCE:
            self.publish_command(self.move_forward)
        elif min(self.right) >= MIN_DISTANCE:
            if min(self.right_front) <= MIN_DISTANCE:
                self.publish_command(self.move_forward)
            elif min(self.right_back) >= MIN_DISTANCE:
                self.publish_command(self.turn_right)
            else:
                self.publish_command(self.slide_right)

    def obstacle_avoidance(self):
        # self.continue_right and self.continue_left is used to
        # keep turning in same direction as robot started
        # otherwise it might get stuck in corners
        rospy.loginfo("Obstacle avoidance")

        if self.continue_right:
            self.keep_right()
        elif self.continue_left:
            self.keep_left()
        elif all(scan_range >= MIN_DISTANCE for scan_range in self.left_front):
            self.keep_left()
        elif all(scan_range >= MIN_DISTANCE for scan_range in self.right_front):
            self.keep_right()
        elif all(scan_range >= MIN_DISTANCE for scan_range in self.left_back):
            self.keep_left()
        elif all(scan_range >= MIN_DISTANCE for scan_range in self.right_back):
            self.keep_right()
        else:
            self.keep_left()

    def random_walk(self):
        rospy.loginfo("Random walk")

        self.continue_right = False
        self.continue_left = False
        is_distance_reached = sqrt((self.current_pose_x - self.last_pose_x)
                                   ** 2
                                   + (self.current_pose_y - self.last_pose_y)
                                   ** 2) <= WALKING_DISTANCE

        # move forward 3 m and turn by random angle
        if is_distance_reached:
            self.publish_command(self.move_forward)
            self.save_position = False
        else:
            self.publish_command(Twist())
            self.save_position = True
            for _ in range(0, random.randint(0, 120)):
                self.last_pose_theta = self.current_pose_theta
                self.last_pose_x = self.current_pose_x
                self.last_pose_y = self.current_pose_y

                self.publish_command(self.turn_left)
                self.rate.sleep()

    def scan_callback(self, scan):
        self.front = scan.ranges[330:] + scan.ranges[0:30]
        self.left_front = scan.ranges[30:90]
        self.left_back = scan.ranges[90:150]
        self.left = scan.ranges[60:120]
        self.back = scan.ranges[150:210]
        self.right_back = scan.ranges[210:270]
        self.right_front = scan.ranges[270:330]
        self.right = scan.ranges[240:300]

        if self.save_position:
            self.last_pose_theta = self.current_pose_theta
            self.last_pose_x = self.current_pose_x
            self.last_pose_y = self.current_pose_y

        # switch between behaviours based on whether an obstacle
        # is detected in front or on right of robot
        if self.is_wall_found:
            self.right_hand_wall_follow()
        else:
            if all(scan_range <= MIN_DISTANCE for scan_range in self.right):
                self.publish_command(Twist())
                self.right_hand_wall_follow()
            elif all(scan_range >= MIN_DISTANCE for scan_range in self.front):
                self.random_walk()
            else:
                self.obstacle_avoidance()

    def odom_callback(self, odom):
        quart = [odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(quart)

        self.current_pose_theta = yaw
        self.current_pose_x = odom.pose.pose.position.x
        self.current_pose_y = odom.pose.pose.position.y


if __name__ == '__main__':
    try:
        Behaviour().start()
    except rospy.ROSInterruptException:
        pass
