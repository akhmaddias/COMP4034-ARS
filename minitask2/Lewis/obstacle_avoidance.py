#!/usr/bin/env python
import rospy
import tf

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from math import radians, pi as PI, sqrt, cos, sin, atan
from random import uniform

FORWARD_SPEED = 0.2
WALK_DISTANCE = 3.0
MIN_DISTANCE = 0.5
TURN_SPEED = ((2 * PI) / 360) * 25  # 25 Deg/s

class Obstacle_avoidance():

    def __init__(self):
        rospy.init_node('Obstacle_avoidance', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # For movement
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.forward = Twist()
        self.forward.linear.x = FORWARD_SPEED
       
        self.turn_left = Twist()
        self.turn_left.linear.x = 0
        self.turn_left.angular.z = TURN_SPEED
       
        self.turn_right = Twist()
        self.turn_right.linear.x = 0
        self.turn_right.angular.z = -TURN_SPEED

        self.turn_stop = Twist()
        self.turn_stop.linear.x = 0
        self.turn_stop.angular.z = 0
       
        self.direction = 0
        self.direction_set = False

        # For behavioural blocking
        self.blocking_publisher = rospy.Publisher('blocking', Int32, queue_size=10)
        self.blocking = False
        self.blocked = False

    def drive_forward(self):
        self.vel_publisher.publish(self.forward)

    def drive_turn_left(self):
        self.vel_publisher.publish(self.turn_left)

    def drive_turn_right(self):
        self.vel_publisher.publish(self.turn_right)

    def drive_stop(self):
        self.vel_publisher.publish(self.turn_stop)

    def scan_callback(self, scan):
        '''
        Handles the scanner data and determines the correct action
        '''
        min_val_angle = 0
        min_val = 50
        for i in range(300, 359):
            if scan.ranges[i] < min_val:
                min_val = scan.ranges[i]
                min_val_angle = i
        for i in range(0, 60):
            if scan.ranges[i] < min_val:
                min_val = scan.ranges[i]
                min_val_angle = i

        if not self.blocked:
            if min_val < MIN_DISTANCE:
                if self.blocking == False:
                    rospy.loginfo("Blocking")
                    self.drive_stop()
                    self.blocking = True
                    self.direction_set = False
                    self.blocking_publisher.publish(1)
                self.obstacle_avoidance(scan, min_val_angle)
            else:
                if self.blocking == True:
                    rospy.loginfo("Not blocking")
                    self.blocking = False
                    self.direction = 0
                    self.direction_set = False
                    self.drive_stop()
                    self.blocking_publisher.publish(0)

    def obstacle_avoidance(self, scan, min_val_angle):
        # Identify where the object is
        if not self.direction_set:
            rospy.loginfo("Obstacle avoidance")
            if min_val_angle < 90:
                self.direction = 2
                self.direction_set = True
            else: 
                self.direction = 1
                self.direction_set = True
        else: 
            if self.direction == 1:
                self.vel_publisher.publish(self.turn_left)
            else: 
                self.vel_publisher.publish(self.turn_right)

    def start(self):
        rospy.loginfo("Starting Obstacle Avoidance")
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)        
        rospy.spin()

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.blocked = True
        self.vel_publisher.publish(Twist())
        rospy.loginfo("Stopping Obstacle Avoidance")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Obstacle_avoidance().start()
    except rospy.ROSInterruptException:
        pass
