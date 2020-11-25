#!/usr/bin/env python
import rospy

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point

FRAME_ID = 'map'
WAIT_TIME = 5

class Navigate ():

    DEFAULT_WP = ((0.0))  # TODO

    def __init__(self, waypoints=Navigate.DEFAULT_WP):
        rospy.init_node('navigation', anonymous=True)
        self.waypoints = waypoints
        self.action = SimpleActionClient('move_base', MoveBaseAction)

    def drive(self):
        waypoint_index = 0
        while not rospy.is_shutdown() and waypoint_index < len(self.waypoints):
            if not self.goto_pos(self.waypoints[waypoint_index]):
                rospy.logerr("Oops, something has gone wrong!")

    def goto_pos(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = FRAME_ID
        goal.target_pose.header.stamp = rospy.time.now()
        goal.target_pose.pose.position = Point(position[0], position[1], 0)
        goal.target_pose.pose.orientation.w = 1.0

        self.action.send_goal(goal)

        while not rospy.is_shutdown():
            self.action.wait_for_result(rospy.Duration(WAIT_TIME))
            state = self.action.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True
            elif state == GoalStatus.ACTIVE:
                rospy.loginfo("Moving")
            else:
                return False


if __name__ == '__main__':
    try:
        navigation = Navigate()
        navigation.drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
