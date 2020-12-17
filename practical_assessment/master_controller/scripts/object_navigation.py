#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Pose
from master_controller.msg import DetectedObject
from sensor_msgs.msg import LaserScan

# control constants
STOP = -1
START = 1
REACHED = 0
DIST_TO_OBJECT = 0.5

# turning controller constants
TURN_GAIN = 0.005
TURN_ERR_THRESH = 50

# moving controller constants
MOVING_ERROR_THRESHOLD = 0.5
MOVE_GAIN = 0.5
MOVE_MAX_SPEED = 0.2
MOVE_MIN_SPEED = 0.01

# laser scan constants
WINDOW = 20
TARGET = 0
OFFSET = 10


class ObjectNavigation():

    def __init__(self):
        # init node
        rospy.init_node('object_navigation', anonymous=False)

        # variables
        self.is_running = False
        self.object_name = None
        self.object_action = None
        self.object_detected = DetectedObject()
        self.dist_to_object = None
        self.is_moving = False
        self.move_cmd = Twist()
        self.stop_cmd = Twist()
        self.stop_cmd.linear.x = 0
        self.stop_cmd.linear.y = 0
        self.stop_cmd.linear.z = 0
        self.stop_cmd.angular.x = 0
        self.stop_cmd.angular.y = 0
        self.stop_cmd.angular.z = 0

        # init subscribers
        self.object_control_sub = rospy.Subscriber("object_control",
                                                   Int32,
                                                   self.object_control_cb)
        self.object_detected_sub = rospy.Subscriber("object_navigation_detected",
                                                    DetectedObject,
                                                    self.object_detected_cb)
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
        for i in range(TARGET - OFFSET + 1, TARGET + OFFSET):
            self.dist_to_object = min(scandata.ranges[i], self.dist_to_object)

    def object_detected_cb(self, data):
        '''
        Saves object position and name
        '''
        self.object_name = data.object_name
        self.object_detected = data
        self.navigation_control()

    def object_control_cb(self, action):
        '''
        Saves the control state and starts navigation
        '''
        self.object_action = action.data

    def navigation_control(self):
        if self.object_action == START:
            self.navigate_to_object()
        elif self.object_action == STOP:
            rospy.loginfo("Stopping object navigation...")
            self.twist_pub.publish(self.stop_cmd)
            self.is_moving = False
        elif self.object_action == REACHED:
            rospy.loginfo("Object reached!")
        else:
            rospy.loginfo("Waiting for the command")

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
        elif not self.object_detected:
            rospy.logerr("Estimated object pose is not received. Waiting...")
            return

        if not self.is_moving:
            rospy.loginfo("Navigation to object {}. Estimated position at ({}, {})"
                          .format(self.object_name,
                                  self.object_detected.x,
                                  self.object_detected.y))
        if self.object_action == 1:
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
        # screen width middle
        width_middle = self.object_detected.size_x / 2
        # turn error
        turn_err = self.object_detected.x - width_middle

        # if error less than threshold then stop turning
        turnspeed = -float(turn_err) * TURN_GAIN

        # target reached
        if turn_err < TURN_ERR_THRESH and movespeed == 0:
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
        self.twist_pub.publish(self.stop_cmd)
        self.object_control_pub.publish(REACHED)


if __name__ == '__main__':
    try:
        ObjectNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
