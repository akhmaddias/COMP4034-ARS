#!usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import radians, sqrt
import random

move_cmd = Twist()
move_cmd.linear.x = 0.2

left_turn_cmd = Twist()
left_turn_cmd.linear.x = 0
left_turn_cmd.angular.z = radians(30)

right_turn_cmd = Twist()
right_turn_cmd.linear.x = 0
right_turn_cmd.angular.z = radians(-30)

stop_cmd = Twist()

save_position = True
last_pose_theta = None
last_pose_x = None
last_pose_y = None

continue_right = False
continue_left = False

front = None
right_front = None
right_back = None
right = None
back = None
left_back = None
left_front = None
left = None

is_wall_found = False

class Behaviour():

    def __init__(self):
        self.behaviour = input("Enter behaviour mode '0' for random walk '1' for right hand wall following: ")
        rospy.init_node('behaviour')

        # Subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

        self.rate = rospy.Rate(10)
        self.current_pose_theta = None
        self.current_pose_x = None
        self.current_pose_y = None
        rospy.spin()

    def publish_command(self, msg):
        self.cmd_vel_pub.publish(msg)

    def turn_left(self):
        global continue_left, continue_right
        continue_right = False
        continue_left = True
        self.publish_command(left_turn_cmd)
        print("Turning left")
    
    def turn_right(self):
        global continue_left, continue_right
        continue_right = True
        continue_left = False
        self.publish_command(right_turn_cmd)
        print("Turning right")

    def right_hand_wall_follow(self):
        rospy.loginfo("Right hand wall following")
        global is_wall_found

        if not is_wall_found:
            if all(i >= 0.35 for i in front):
                self.publish_command(move_cmd)
            else:
                is_wall_found = True
                self.publish_command(Twist())
        else:
            if all(i <= 0.25 for i in right) and all(i <= 0.35 for i in front):
                self.publish_command(left_turn_cmd)
            elif all(i <= 0.25 for i in right) and all(i >= 0.35 for i in front):
                self.publish_command(move_cmd)
            elif all(i >= 0.25 for i in right) and all(i <= 0.35 for i in front):
                self.publish_command(left_turn_cmd)
            else:
                self.publish_command(right_turn_cmd)

    def obstacle_avoidance(self):
        # continue_right and continue_left is used to
        # keep turning in same direction as robot started
        # otherwise it might get stuck in corners
        if continue_right:
            self.turn_right()
        elif continue_left:
            self.turn_left()
        elif all(i >= 0.25 for i in left_front):
            self.turn_left()
        elif all(i >= 0.25 for i in right_front):
            self.turn_right()
        elif all(i >= 0.25 for i in left_back):
            self.turn_left()
        elif all(i >= 0.25 for i in right_back):
            self.turn_right()
        else:
            self.turn_left()

    def random_walk(self):
        rospy.loginfo("Random walk")
        global save_position, continue_right, continue_left
        global last_pose_theta, last_pose_x, last_pose_y

        continue_right = False
        continue_left = False

        # move forward 3 m and turn by random angle
        if sqrt((self.current_pose_x - last_pose_x) ** 2 +
                (self.current_pose_y - last_pose_y) ** 2) < 3:
            self.publish_command(move_cmd)
            save_position = False
        else:
            self.publish_command(Twist())
            save_position = True
            for x in range(0, random.randint(0, 120)):
                last_pose_theta = self.current_pose_theta
                last_pose_x = self.current_pose_x
                last_pose_y = self.current_pose_y

                self.publish_command(left_turn_cmd)
                self.rate.sleep()

    def scan_callback(self, scan):
        global front, right_front, right_back, back, left_back, left_front, right, left
        global last_pose_theta
        global last_pose_x
        global last_pose_y
        
        front = scan.ranges[330:] + scan.ranges[0:30]
        left_front = scan.ranges[30:90]
        left_back = scan.ranges[90:150]
        left = scan.ranges[60:120]
        back = scan.ranges[150:210]
        right_back = scan.ranges[210:270]
        right_front = scan.ranges[270:330]
        right = scan.ranges[240:300]

        if self.behaviour == 1:
            # this behaviour is not working properly for now
            self.right_hand_wall_follow()
        else:
            if save_position:
                last_pose_theta = self.current_pose_theta
                last_pose_x = self.current_pose_x
                last_pose_y = self.current_pose_y
                
            # switch between behaviours based on whether an obstacle
            # is detected in front of robot
            if all(i >= 0.35 for i in front):
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
        Behaviour()
    except rospy.ROSInterruptException:
        pass