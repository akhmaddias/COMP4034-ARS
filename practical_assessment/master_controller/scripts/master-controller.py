#!/usr/bin/env python
'''
The root controller class for the Turtlebot3 Waffle.
This implements an object search behaviour that enables the robot to search
for predefined objects visible in the robot’s camera. The controller selects
the most appropriate behaviour based on what is occuring.

Behaviour priority is as follows:
    ~ Recovery behaviour (to escape from being wedged or trapped)
    ~ Collision avoidance (to avoid colliding into items in the scene)
    ~ Navigation to objects (to move from the current position to the object of interest)
    ~ Object detection (to highlight an object of interest and trigger navigation to object)
    ~ Map navigation (drives the robot between waypoints in order to
        navigate the environment)
    ~ Random walk (if no waypoints have been given, walk randomly until something happens)
'''

__author__      = "Lewis C Brand"

import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, String

#  Overall parameters
WAYPOINTS = () #  Waypoints format (X, Y, Z, room_number)
OBJECTS = (
    "fire hydrant", 
    "green box", 
    "mail box",
    "number 5"
    )

#  State flags
ACTIVE_RUNNING = 1
ACTIVE_PAUSED = 2
ACTIVE_STOPPED = -1
IGNORED = 3
INACTIVE = 0

#  Behaviour
NONE = -1
MAPPING = 0
OBJECT_NAVIGATION = 1
COLLISION_AVOIDANCE = 2
RECOVERY = 3

class Controller():
    '''
    The controller class
    '''

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.shutting_down = False

        rospy.loginfo("Initialising state model")
        #  State variables
        self.state_mapping = INACTIVE
        self.state_object_detection = INACTIVE
        self.state_object_navigation = INACTIVE
        self.state_collision_avoidance = INACTIVE
        self.state_recovery = INACTIVE

        #  Behavioural
        self.behaviour_current = NONE
        self.behaviour_previous = NONE

        #  Navigation
        self.waypoints = []
        self.waypoints_visited = 0
        self.waypoint_current = 0
        self.room = 0
        self.objects_found = 0

        #  Goal tracking
        self.objects = []
        self.object_current = 0
        self.objects_found = 0
        self.objects_total = len(OBJECTS)

        self.init_waypoints()
        self.init_objects()

        self.init_pub_subs()

        self.controller()


    def init_waypoints(self):
        '''
        Initialises waypoint array
        '''
        rospy.loginfo("Initialising waypoints")
        i = 0
        for waypoint in WAYPOINTS:
            waypoint_details = {}
            waypoint_details["id"] = i
            waypoint_details["x"] = waypoint[0]
            waypoint_details["y"] = waypoint[1]
            waypoint_details["z"] = waypoint[2]
            waypoint_details["room"] = waypoint[3]
            waypoint_details["attempt"] = 0
            waypoint_details["visited"] = False
            waypoint_details["re-visit"] = False

            self.waypoints[i] = waypoint_details
            i += 1


    def init_objects(self):
        '''
        Initialises objects array
        '''
        rospy.loginfo("Initialising objects")
        i = 0
        for detectable_object in OBJECTS:
            object_details = {}
            object_details["id"] = i
            object_details["x"] = 0.0
            object_details["y"] = 0.0
            object_details["name"] = detectable_object
            object_details["visited"] = False
            object_details["visiting"] = False

            self.objects[i] = object_details
            i += 1


    def init_pub_subs(self):
        rospy.loginfo("Initialising publishers and subscribers")
        #  Mapping pub/subs
        self.mapping_pose_publisher = rospy.Publisher(
            'mapping_pose', Pose, queue_size=10)
        self.mapping_control_publisher = rospy.Publisher(
            'mapping_control', Int32, queue_size=10)
        self.mapping_control_subscriber = rospy.Subscriber(
            'mapping_control', Int32, self.mapping_control_callback)

        #  Object detection subs
        self.object_detection_type_subscriber = rospy.Subscriber(
            'object_detection_type', String, self.object_detection_type_callback)
        self.object_detection_pose_subscriber = rospy.Subscriber(
            'object_detection_pose', Pose, self.object_detection_pose_callback)

        #  Collission avoidance pub/subs
        self.collision_control_publisher = rospy.Publisher(
            'collision_control', Int32, queue_size=10)
        self.collision_control_subscriber = rospy.Subscriber(
            'collision_control', Int32, self.collision_control_callback)

        #  Recovery behaviour pub/subs
        self.recovery_control_publisher = rospy.Publisher(
            'recovery_control', Int32, queue_size=10)
        self.recovery_control_subscriber = rospy.Subscriber(
            'recovery_control', Int32, self.recovery_control_callback)

        #  Object navigation behaviour pub/subs
        self.object_navigation_type_publisher = rospy.Publisher(
            'object_navigation_type', String, queue_size=10)
        self.object_navigation_pose_publisher = rospy.Publisher(
            'object_navigation_pose', Pose, queue_size=10)
        self.object_navigation_control_publisher = rospy.Publisher(
            'object_control', Int32, queue_size=10)
        self.object_pose_subscriber = rospy.Subscriber(
            'object_position', Pose, self.object_position_callback)
        self.object_control_subscriber = rospy.Subscriber(
            'object_control', Int32, self.object_control_callback)


    def controller(self):
        pass


    def mapping_control_callback(self):
        pass


    def object_detection_type_callback(self):
        pass


    def object_detection_pose_callback(self):
        pass


    def collision_control_callback(self):
        pass


    def recovery_control_callback(self):
        pass


    def object_position_callback(self):
        pass


    def object_control_callback(self):
        pass


    def mapping_send_coordinates(self, waypoint):
        pose = Pose()
        pose.position.x = waypoint[0]
        pose.position.y = waypoint[1]
        pose.position.z = waypoint[2]
        self.mapping_pose_publisher.publish(pose)


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

    rospy.init_node('master-controller', anonymous=True)
    Controller()
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
