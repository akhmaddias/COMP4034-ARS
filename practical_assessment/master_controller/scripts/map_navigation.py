#!/usr/bin/env python

__author__ = "Ben Henaghan"
__author__ = "Lewis C Brand"


import rospy
import actionlib

import numpy as np

from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose


ACTION_START = 1
ACTION_PAUSE = 2
ACTION_STOP = -1
ACTION_SUCCESS = 0
ACTION_FAILED = 5

class MapNavigator():

    def __init__(self):
        rospy.init_node('map_navigator', anonymous=False)
        self.paused = False
        self.stopped = True
        self.target = None

        self.action_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.client_goal_handle = None
        self.success_pub = rospy.Publisher("mapping_control", Int32, queue_size=10)
        self.control_sub = rospy.Subscriber("mapping_control", Int32, self.control_cb)
        self.pose_subscriber = rospy.Subscriber("mapping_pose", Pose, self.target_cb)

    def control_cb(self, command):
        rospy.loginfo("Received control message")
        if command.data == ACTION_PAUSE:
            rospy.loginfo("Pausing navigation to waypoint")
            self.paused = True
            self.stopped = False
            self.action_client.cancel_all_goals()
        elif command.data == ACTION_START:
            rospy.loginfo("Starting navigation to waypoint")
            self.paused = False
            self.stopped = False
            self.goto_target()
        else:  # Stop
            rospy.loginfo("Stopping navigation to waypoint")
            self.stopped = True
            self.action_client.cancel_all_goals()

    def target_cb(self, pose):
        rospy.loginfo("Received coordinates message")
        # self.stopped = False
        self.target = (pose.position.x, pose.position.y, 0)
        # self.goto_target()

    def goto_target(self):
        if self.stopped:
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(*self.target)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.action_client.send_goal(goal)
        rospy.loginfo('Sent move goal of x:{:6.1f} y:{:6.1f} z:{:6.1f}'.
                      format(*self.target))

    def check_for_success(self):
        rospy.loginfo('Mapping active')
        while not rospy.is_shutdown():
            if self.stopped or self.paused:
                continue
            self.action_client.wait_for_result(rospy.Duration(5))
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Waypoint reached')
                self.success_pub.publish(ACTION_SUCCESS)
                self.stopped = True
            elif self.action_client.get_state() != GoalStatus.ACTIVE:
                if not self.paused:
                    rospy.logerr("Failed to reach target at {}".format(self.target))
                    self.success_pub.publish(ACTION_FAILED)
                    self.stopped = True
                

def main():
    '''
    Is initially called, inits the node and starts the class.
    '''
    MapNavigator().check_for_success()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
