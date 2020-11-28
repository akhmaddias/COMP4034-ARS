#!/usr/bin/env python
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

# coordinates are taken manually from map
BOTTOM_RIGHT_ROOM = (-6.1, -2.6)
BOTTOM_LEFT_ROOM = (-6.1, 3.4)
BOTTOM_HALL = (-2.6, 2.5)
MIDDLE_SMALL_ROOM = (1.0, 2.7)
TOP_HALL = (5.0, 2.7)
TOP_RIGHT_ROOM = (6.1, -4.4)

POINTS = (BOTTOM_RIGHT_ROOM, BOTTOM_LEFT_ROOM, BOTTOM_HALL,
          MIDDLE_SMALL_ROOM, TOP_HALL, TOP_RIGHT_ROOM)


class MapNavigation():

    def __init__(self):
        rospy.init_node('map_navigation', anonymous=False)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

        # define a client for to send goal requests to the move_base server
        # through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # count number of waypoints reached
        self.waypoints_count = 1

    def start(self):
        # wait for the action server to come up
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
        for point in POINTS:
            rospy.loginfo("Moving to waypoint #%s", self.waypoints_count)
            self.waypoints_count += 1
            self.move_to_goal(point)
        rospy.signal_shutdown("All waypoints reached. Shutting down.")

    def move_to_goal(self, point):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position = Point(point[0], point[1], 0)
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        self.ac.send_goal(self.goal)

        self.ac.wait_for_result(rospy.Duration(60))

        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("You have reached the waypoint")
            return True
        else:
            rospy.loginfo("The robot failed to reach the waypoint")
            return False


if __name__ == '__main__':
    try:
        MapNavigation().start()
    except rospy.ROSInterruptException:
        pass
