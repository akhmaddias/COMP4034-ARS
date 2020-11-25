#!/usr/bin/env python
'''
Will move a Turtlebot3 Waffle between set waypoints using move_base
'''

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point

WAYPOINTS = (
    (5.5, -4.5),  # Top right room
    (5.0, 4.0),  # Top left room
    (1.0, 3.0),  # Small middle room
    (-2.0, 4.0),  # Otherside of starting room
    (-6.0, 2.0),  # Bottom left room
    (-6.0, -3.0),  # Bottom right room
    (-3.0, 1.0)  # Return to starting position
    )

class Patrol():
    '''
    Will move a Turtlebot3 Waffle between set waypoints using move_base
    '''

    def __init__(self):
        # Used to send goal requests to the move_base server
        self.action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.on_shutdown(self.shutdown)
        self.shutting_down = False

        self.waypoints_visited = 0
        self.waypoints = WAYPOINTS

        self.drive()

    def drive(self):
        '''
        Loops through each waypoint and sets coordinates to travel to
        '''

        while not self.action_client.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for move_base server...")

        while not self.shutting_down:
            for waypoint in self.waypoints:
                rospy.loginfo("Moving to waypoint %s %s. Visited %s waypoints so far",
                    self.waypoints_visited + 1, waypoint, self.waypoints_visited)

                self.waypoints_visited += 1
                arrived_at_waypoint = self.go_to_waypoint(waypoint)
                if not arrived_at_waypoint:
                    rospy.loginfo("Exiting waypoint loop")
                    break

            rospy.loginfo("Exiting patrol loop")
        self.shutdown()

    def go_to_waypoint(self, waypoint):
        '''
        Given a waypoint, prepares and sends the coordinates
        to the move_base server monitors progress
        '''

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(waypoint[0], waypoint[1], 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        #goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal coordinates (%f,%f) to move_base server",
                        waypoint[0], waypoint[1])

        self.action_client.send_goal(goal)
        rospy.loginfo("Navigating to waypoint %s", self.waypoints_visited)

        while not self.shutting_down:
            self.action_client.wait_for_result(rospy.Duration(5))
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Waypoint %s reached (%f,%f)", self.waypoints_visited,
                            waypoint[0], waypoint[1])
                return True
            elif self.action_client.get_state() != GoalStatus.ACTIVE:
                rospy.loginfo("Failed to reach the waypoint %s", self.waypoints_visited)
                rospy.loginfo("Goal status: %s", GoalStatus)
                return False

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.shutting_down = True  # Terminates continuous loops
        rospy.loginfo("Stopping")
        rospy.sleep(1)

def main():
    '''
    Is initially called, inits the node and starts the class
    '''

    rospy.init_node('patrol', anonymous=True)
    Patrol()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Stopping")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
