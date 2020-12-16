import rospy
import actionlib

import numpy as np

from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point


ACTION_START = 1
ACTION_PAUSE = 2
ACTION_STOP = -1
ACTION_SUCCESS = 0

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

    def control_cb(self, command):
        if command == ACTION_PAUSE:
            self.paused = True
            self.stopped = False
            self.action_client.cancel_all_goals()
        elif command == ACTION_START:
            self.paused = False
            self.stopped = False
            self.goto_target()
        else:  # Stop
            self.stopped = True
            self.action_client.cancel_all_goals()


    def target_cb(self, pose):
        self.stopped = False
        self.target = (pose.x, pose.y, 0)
        self.goto_target()

    def goto_target(self):
        if self.stopped:
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(*self.target)

        rospy.loginfo('Sending move goal of x:{:6.1f} y:{:6.1f} z:{:6.1f}'.
                      format(*self.target))

    def check_for_success(self):
        while not rospy.is_shutdown():
            if self.stopped:
                continue
            self.action_client.wait_for_result(rospy.Duration(5))
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Waypoint reached')
                self.success_pub.publish(ACTION_SUCCESS)
            elif self.action_client.get_state() != GoalStatus.ACTIVE:
                rospy.logerr("Failed to reach target at {}".format(self.target_pose))