#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from master_controller.msg import DetectedObject
from sensor_msgs.msg import LaserScan
from math import radians

# control constants
STOP = -1
START = 1
PAUSE = 2
REACHED = 0
DIST_TO_OBJECT = 0.5

# turning controller constants
TURNING_ERROR_THRESHOLD = radians(10)
TURN_GAIN = 0.5
TURN_MAX_SPEED = 1
TURN_MIN_SPEED = 0.01

# moving controller constants
MOVING_ERROR_THRESHOLD = 0.5
MOVE_GAIN = 0.5
MOVE_MAX_SPEED = 0.3
MOVE_MIN_SPEED = 0.01

# laser scan constants
WINDOW = 30
TARGET = 0
OFFSET = 15


class ObjectNavigation():

    def __init__(self):
        # init node
        rospy.init_node('object_navigation', anonymous=False)

        # variables
        self.is_running = False
        self.object_name = None
        self.object_pose = None
        self.object_action = None
        self.curr_position = None
        self.curr_orientation = None
        self.target_orientation = None
        self.dist_to_object = None
        self.scan_index = None
        self.is_moving = False
        self.move_cmd = Twist()

        # init subscribers
        self.object_control_sub = rospy.Subscriber("object_control",
                                                   Int32,
                                                   self.object_control_cb)
        self.object_detected_sub = rospy.Subscriber("object_navigation_detected",
                                                    DetectedObject,
                                                    self.object_detected_cb)
        self.amcl_sub = rospy.Subscriber("amcl_pose",
                                         PoseWithCovarianceStamped,
                                         self.amcl_cb)
        self.scan_sub = rospy.Subscriber("scan",
                                         LaserScan,
                                         self.scan_cb)

        # init publishers
        self.object_control_pub = rospy.Publisher("object_control",
                                                  Int32, queue_size=1)
        self.object_position_pub = rospy.Publisher("object_position",
                                                   Pose, queue_size=1)
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    def scan_cb(self, scandata):
        '''
        Saves laser scan data
        '''
        self.dist_to_object = scandata.ranges[TARGET - OFFSET]
        self.scan_index = TARGET
        for i in range(TARGET - OFFSET + 1, TARGET + OFFSET):
            self.dist_to_object = min(scandata.ranges[i], self.dist_to_object)
            self.scan_index = i if self.dist_to_object > scandata.ranges[i] else self.scan_index

    def object_detected_cb(self, data):
        '''
        Saves object position and name
        '''
        self.object_name = data.object_name
        self.object_pose = data

    def amcl_cb(self, amcl):
        '''
        Saves current position and orientation of robot
        '''
        self.curr_position = amcl.pose.pose.position
        quart = [amcl.pose.pose.orientation.x,
                 amcl.pose.pose.orientation.y,
                 amcl.pose.pose.orientation.z,
                 amcl.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quart)
        self.curr_orientation = yaw

    def object_control_cb(self, action):
        '''
        Saves the control state and starts navigation
        '''
        self.object_action = action.data
        if self.object_action == START:
            self.navigate_to_object()
        elif self.object_action == STOP:
            rospy.loginfo("Stopping object navigation...")
            self.move_cmd(Twist())
            self.is_moving = False
        elif self.object_action == REACHED:
            rospy.loginfo("Object reached!")
        else:
            rospy.loginfo("Invalid command. Waiting...")

    def navigate_to_object(self):
        '''
        Checks if object name and pose is received and starts
        navigating to object
        '''
        if not self.is_moving:
            rospy.loginfo("Starting object navigation...")

        if not self.object_name:
            rospy.logerr("Object name is not received. Waiting...")
            return
        elif not self.object_pose:
            rospy.logerr("Estimated object pose is not received. Waiting...")
            return

        if not self.is_moving:
            rospy.loginfo("Navigation to object {}. Estimated position at ({}, {})"
                          .format(self.object_name,
                                  self.object_pose.x,
                                  self.object_pose.y))
        self.calculate_movement()

    def calculate_movement(self):
        '''
        Calculates and moves to the point 0.5m away from object
        on the line between object and robot
        '''
        # move error is set to the distance between object and target point
        move_err = self.dist_to_object

        # if error less than threshold then stop moving
        if move_err <= MOVING_ERROR_THRESHOLD:
            movespeed = 0
        else:
            # else clamp movespeed between min and max values
            movespeed = min(max(move_err * MOVE_GAIN,
                                MOVE_MIN_SPEED),
                            MOVE_MAX_SPEED)

        turn_err = abs(self.scan_index)

        # if error less than threshold then stop turning
        if turn_err <= TURNING_ERROR_THRESHOLD:
            turnspeed = 0
        else:
            # else clamp turnspeed between min and max values
            turnspeed = min(max(turn_err * TURN_GAIN,
                                TURN_MIN_SPEED),
                            TURN_MAX_SPEED)
            turnspeed = turnspeed if self.scan_index < 0 else -turnspeed

        # target reached
        if turnspeed == 0 and movespeed == 0:
            self.goal_reached()
            return

        # set twist command parameters
        self.move_cmd.linear.x = movespeed
        self.move_cmd.angular.z = turnspeed

        self.is_moving = True
        rospy.loginfo("Moving...")
        rospy.loginfo("Movespeed: {}".format(movespeed))
        rospy.loginfo("Turnspeed: {}".format(turnspeed))

        # publish to /cmd_vel topic
        self.twist_pub.publish(self.move_cmd)

    def goal_reached(self):
        '''
        Publishes 0 to /object_control to indicate that object is reached
        '''
        self.is_moving = False
        rospy.loginfo("Reached object {}".format(self.object_name))
        self.twist_pub.publish(Twist())
        self.object_control_pub.publish(REACHED)


if __name__ == '__main__':
    try:
        ObjectNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
