#!usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import radians, sqrt
import random

move_cmd = Twist()
move_cmd.linear.x = 0.1

turn_cmd = Twist()
turn_cmd.linear.x = 0
turn_cmd.angular.z = radians(30)

stop_cmd = Twist()

class Behaviour():

    def __init__(self):
        rospy.init_node('behaviour')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.rate = rospy.Rate(10)
        self.ranges = []
        self.current_pose_theta = None
        self.current_pose_x = None
        self.current_pose_y = None

    def spin(self):
        rospy.loginfo("Program start")
        while not rospy.is_shutdown():
            if len(self.ranges) == 360:
                self.random_walk()
            self.rate.sleep()

    def publish_command(self, msg):
        self.cmd_vel_pub.publish(msg)

    def obstacle_avoidance(self):
        rospy.loginfo("Obstacle avoidance behaviour activated")
        while self.ranges[0] < 0.25 or self.ranges[45] < 0.25 or self.ranges[325] < 0.25:
            self.publish_command(turn_cmd)
            self.rate.sleep()
        self.publish_command(Twist())


    def random_walk(self):
        rospy.loginfo("Random walk behaviour activated")

        last_pose_theta = self.current_pose_theta
        last_pose_x = self.current_pose_x
        last_pose_y = self.current_pose_y

        while self.ranges[0] > 0.25 or self.ranges[45] > 0.25 or self.ranges[325] > 0.25:
            if sqrt((self.current_pose_x - last_pose_x) ** 2 +
                    (self.current_pose_y - last_pose_y) ** 2) < 3:
                self.publish_command(move_cmd)
            else:
                self.publish_command(Twist())
                for x in range(0, random.randint(0, 12)):
                    last_pose_theta = self.current_pose_theta
                    last_pose_x = self.current_pose_x
                    last_pose_y = self.current_pose_y

                    self.publish_command(turn_cmd)
                    self.rate.sleep()
            self.rate.sleep()
        rospy.loginfo(self.ranges[0])
        self.publish_command(Twist())
        self.obstacle_avoidance()

    def scan_callback(self, scan):
        self.ranges = scan.ranges
    
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
        Behaviour().spin()
    except rospy.ROSInterruptException:
        pass