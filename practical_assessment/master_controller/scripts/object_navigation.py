#!/usr/bin/env python
import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sqrt

STOP = -1
START = 1
PAUSE = 2
REACHED = 0
DIST_TO_OBJECT = 0.5


class ObjectNavigation():

    def __init__(self):
        # init node
        rospy.init_node('object_navigation', anonymous=False)

        # variables
        self.is_running = False
        self.object_type = None
        self.object_pose = None
        self.object_action = None
        self.curr_position = None
        self.curr_orientation = None
        self.is_goal_sent = False

        # init subscribers
        self.object_type_sub = rospy.Subscriber("/object_navigation_type",
                                                String, self.object_type_cb)
        self.object_pose_sub = rospy.Subscriber("/object_navigation_pose",
                                                Pose, self.object_pose_cb)
        self.odom_sub = rospy.Subscriber("/amcl_pose",
                                         PoseWithCovarianceStamped,
                                         self.amcl_cb)

        # init publishers
        self.object_control_pub = rospy.Publisher("/object_control",
                                                  Int32, queue_size=1)
        self.object_position_pub = rospy.Publisher("/object_position",
                                                   Pose, queue_size=1)

        # init SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def start(self):
        '''
        Starts main control logic
        '''
        rospy.loginfo("Object navigation node")
        self.object_control_sub = rospy.Subscriber("/object_control",
                                                   Int32,
                                                   self.object_control_cb)
        rospy.spin()

    def object_type_cb(self, object_type):
        self.object_type = object_type.data

    def object_pose_cb(self, pose):
        self.object_pose = pose.position

    def amcl_cb(self, amcl):
        self.curr_position = amcl.pose.pose.position
        self.curr_orientation = amcl.pose.pose.orientation

    def object_control_cb(self, action):
        self.object_action = action.data
        if self.object_action == START:
            if not self.is_goal_sent:
                self.navigate_to_object()
        elif self.object_action == STOP:
            if self.is_goal_sent:
                rospy.loginfo("Stopping object navigation...")
                self.ac.cancel_goal()
            self.is_goal_sent = False
        elif self.object_action == REACHED:
            rospy.loginfo("Object reached!")
        else:
            rospy.loginfo("Invalid command. Waiting...")

    def navigate_to_object(self):
        '''
        Checks if object type and pose is received and starts
        navigating to object
        '''
        rospy.loginfo("Starting object navigation...")
        if not self.object_type:
            rospy.logerr("Object type is not received. Waiting...")
            return
        elif not self.object_pose:
            rospy.logerr("Estimated object pose is not received. Waiting...")
            return

        rospy.loginfo("Navigation to object {}. Estimated position at ({}, {})"
                      .format(self.object_type,
                              self.object_pose.x,
                              self.object_pose.y))
        self.move_to_goal()

    def move_to_goal(self):
        '''
        Calculate and move to the point 0.5m away from object
        on the line between object and robot
        '''
        if not self.curr_position:
            return

        # Current distance between robot and object
        curr_dist = sqrt((self.object_pose.x - self.curr_position.x) ** 2 +
                         (self.object_pose.y - self.curr_position.y) ** 2)

        if (curr_dist <= 0.5):
            self.object_control_pub.publish(REACHED)
            return

        # ratio of required distance to object to current distance
        ratio = DIST_TO_OBJECT / curr_dist

        # calculate x and y coordinates of the point 0.5m away from object
        x = (1 - ratio) * self.object_pose.x + ratio * self.curr_position.x
        y = (1 - ratio) * self.object_pose.y + ratio * self.curr_position.y

        # goal parameters
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation = self.curr_orientation

        # send goal
        self.is_goal_sent = True
        rospy.loginfo("Sending goal to reach point ({}, {})...".format(x, y))
        self.ac.send_goal(goal, done_cb=self.goal_reached_cb)

    def goal_reached_cb(self):
        self.is_goal_sent = False
        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached object {}".format(self.object_type))
            self.object_control_pub.publish(REACHED)
        else:
            rospy.logerr("Failed to reach object {} will try again"
                         .format(self.object_type))\


if __name__ == '__main__':
    try:
        ObjectNavigation().start()
    except rospy.ROSInterruptException:
        pass
