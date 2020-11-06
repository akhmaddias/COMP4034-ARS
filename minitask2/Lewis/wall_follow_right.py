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

class Right_wall_follow():

    def __init__(self):
        rospy.init_node('Right_wall_follow', anonymous=True)
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
        min_val_right = 50
        for i in range(70, 110):
            if scan.ranges[i] < min_val_right:
                min_val_right = scan.ranges[i]

        min_val_right_front = 50
        for i in range(30, 70):
            if scan.ranges[i] < min_val_right_front:
                min_val_right_front = scan.ranges[i]

        min_val_right_rear = 50
        for i in range(110, 140):
            if scan.ranges[i] < min_val_right_rear:
                min_val_right_rear = scan.ranges[i]

        if not self.blocked:
            if min_val_right < MIN_DISTANCE:
                if self.blocking == False:
                    rospy.loginfo("Blocking")
                    self.drive_stop()
                    self.blocking = True
                    self.direction_set = False
                    self.blocking_publisher.publish(1)
                    self.blocking_publisher.publish(3)
                self.wall_follow(scan, min_val_right, min_val_right_front, min_val_right_rear)
            else:
                if self.blocking == True:
                    rospy.loginfo("Not blocking")
                    self.blocking = False
                    self.direction = 0
                    self.direction_set = False
                    self.drive_stop()
                    self.blocking_publisher.publish(0)
                    self.blocking_publisher.publish(2)

    def wall_follow(self, scan, min_val, min_val_front, min_val_rear):
        # Identify where the object is
        if min_val <= MIN_DISTANCE:
            self.vel_publisher.publish(self.forward)
        elif min_val >= MIN_DISTANCE:
            if min_val_front <= MIN_DISTANCE:
                self.vel_publisher.publish(self.forward)
            elif min_val_rear >= MIN_DISTANCE:
                self.vel_publisher.publish(self.turn_right)
            else:
                custom = Twist()
                custom.linear.x = 0.1
                custom.angular.z = TURN_SPEED/2
                self.vel_publisher.publish(custom)

    def start(self):
        rospy.loginfo("Starting Right Wall Follower")
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)        
        rospy.spin()

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.blocked = True
        self.vel_publisher.publish(Twist())
        rospy.loginfo("Stoping Right Wall Follower")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Right_wall_follow().start()
    except rospy.ROSInterruptException:
        pass
