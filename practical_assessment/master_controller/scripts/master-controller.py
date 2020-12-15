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
MAX_ATTEMPTS_PER_WAYPOINT = 5
LOOPING = False

#  State flags
STATE_ACTIVE_RUNNING = 1
STATE_ACTIVE_PAUSED = 2
STATE_ACTIVE_STOPPED = -1
STATE_IGNORED = 3
STATE_INACTIVE = 0

#  Behaviour
BEHAVIOUR_NONE = -1
BEHAVIOUR_MAPPING = 0
BEHAVIOUR_OBJECT_NAVIGATION = 1
COLLISION_AVOIDANCE = 2
BEHAVIOUR_RECOVERY = 3
BEHAVIOUR_OBJECT_DETECTION = 4

#  Control
ACTION_START = 1
ACTION_PAUSE = 2
ACTION_STOP = -1
REACHED_WAYPOINT = 0
REACHED_OBJECT = 0
COLLISION_TO_AVOID = 1
NO_COLLISION_TO_AVOID = 0
IN_RECOVERY = 1
NO_IN_RECOVERY = 0


class Controller():
    '''
    The controller class
    '''

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.shutting_down = False

        rospy.loginfo("Initialising state model")
        #  State variables
        self.state_mapping = STATE_INACTIVE
        self.state_object_detection = STATE_INACTIVE
        self.state_object_navigation = STATE_INACTIVE
        self.state_collision_avoidance = STATE_INACTIVE
        self.state_recovery = STATE_INACTIVE

        #  Behavioural
        self.behaviour_current = BEHAVIOUR_NONE
        self.behaviour_previous = BEHAVIOUR_NONE

        #  Navigation
        self.waypoints = []
        self.waypoints_visited = 0
        self.waypoints_skipped = 0
        self.waypoint_current = 0
        self.room = 0
        self.objects_found = 0

        #  Goal tracking
        self.objects = []
        self.object_current = 0
        self.objects_found = 0
        self.objects_total = len(OBJECTS)
        self.objects_mapping_aware = 0

        self.init_waypoints()
        self.init_objects()

        self.init_pub_subs()

        self.change_behaviour(BEHAVIOUR_NONE, 0, BEHAVIOUR_MAPPING, STATE_ACTIVE_RUNNING)
        self.mapping_run()


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
        '''
        Registers all necessary publishers and subscribers in order to communicate with the appropriate nodes.
        '''
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


    def change_behaviour(self, current_behaviour, current_behaviour_new_state, new_behaviour, new_behaviour_new_state):
        '''
        Signals a change in current behaviour and changes states appropriately.
        '''
        rospy.loginfo("Behaviour changed from " + 
            self.get_behaviour_name(current_behaviour) + 
            " to " + 
            self.get_behaviour_name(new_behaviour))
        self.behaviour_previous = current_behaviour
        self.behaviour_current = new_behaviour

        self.change_behaviour_state(current_behaviour, current_behaviour_new_state)
        self.change_behaviour_state(new_behaviour, new_behaviour_new_state)


    def change_behaviour_state(self, behaviour, new_state):
        '''
        Changes the current state of a behaviour.
        '''
        if behaviour == BEHAVIOUR_MAPPING:
            rospy.loginfo("" + self.get_behaviour_name(behaviour) + 
            " changed state from " + 
             + self.state_mapping + 
            " to " + new_state)
            self.state_mapping = new_state
        elif behaviour == BEHAVIOUR_OBJECT_NAVIGATION:
            rospy.loginfo("" + self.get_behaviour_name(behaviour) + 
            " changed state from " + 
             + self.state_object_navigation + 
            " to " + new_state)
            self.state_object_navigation = new_state
        elif behaviour == BEHAVIOUR_RECOVERY:
            rospy.loginfo("" + self.get_behaviour_name(behaviour) + 
            " changed state from " + 
             + self.state_recovery + 
            " to " + new_state)
            self.state_recovery = new_state


    def get_behaviour_name(self, behaviour):
        '''
        Given a behaviour, returns a string of the name of the behaviour.
        '''
        behaviour_string = ""
        if behaviour == BEHAVIOUR_MAPPING:
            behaviour_string = "Mapping"
        elif behaviour == BEHAVIOUR_OBJECT_NAVIGATION:
            behaviour_string = "Object navigation"
        elif behaviour == BEHAVIOUR_RECOVERY:
            behaviour_string = "Recovery behaviour"
        elif behaviour == BEHAVIOUR_NONE:
            behaviour_string = "None"
        return behaviour_string


    def return_to_previous_behaviour(self):
        '''
        Signals a return from the current behaviour to the finishing one and changes
        the states appropriately.
        '''
        pass


    def mapping_send_coordinates(self, waypoint):
        '''
        Translates and then sends a waypoint to the mapping node
        '''
        pose = Pose()
        pose.position.x = waypoint["x"]
        pose.position.y = waypoint["y"]
        pose.position.z = waypoint["z"]
        self.mapping_pose_publisher.publish(pose)


    def mapping_run(self):
        '''
        Method used to instruct the mapping node to perform a task.
        '''
        self.mapping_send_coordinates(self.waypoints[self.waypoint_current])
        self.mapping_control_publisher.publish(ACTION_START)


    def mapping_resume(self):
        '''
        Resumes the mapping function from paused or stopped.
        If mapping was paused, just resume the previous operation. If mapping was stopped
        however, then an exceptional circumstance was just recoved from. If stopped due to
        an object, the rest of the room is irrelevant and so mapping proceeds to the next
        room. If stopped due to a false hit for an object or because the recovery behaviour
        was initiated, then the current waypoint needs to be treated with caution. The attempt
        counter us used to do this. By increasing the attempt number, the robot will attempt
        to avoid waypoints that cause it difficulty. After a waypoint has exceeded the attempt
        threshold, the waypoint is skipped and the next waypoint in the list is chosen. 
        '''
        previous_state = self.state_mapping

        if previous_state == STATE_ACTIVE_PAUSED:
            self.change_behaviour(BEHAVIOUR_MAPPING, STATE_ACTIVE_RUNNING, self.behaviour_previous, STATE_INACTIVE)
            self.mapping_control_publisher.publish(ACTION_START)

        elif previous_state == STATE_ACTIVE_STOPPED:
            if self.objects_found > self.objects_mapping_aware:  # New object found
                if self.objects_found >= self.objects_total:  # If there are no more objects left
                    self.mapping_control_publisher(ACTION_STOP)
                    self.change_behaviour(BEHAVIOUR_MAPPING, STATE_INACTIVE, BEHAVIOUR_NONE, 0)
                    self.all_objects_found()

                else:  # If resuming, objects found but still more to go 
                    self.skip_remaining_room_waypoints()  # Go to the next room
                    self.go_to_next_waypoint()
                    
            elif (self.waypoints[self.waypoint_current]["attempt"] < MAX_ATTEMPTS_PER_WAYPOINT):  # If no objects found and there are attempts remaining on the current waypoint 
                self.waypoints[self.waypoint_current]["attempt"] =+ 1
                self.change_behaviour(BEHAVIOUR_MAPPING, STATE_ACTIVE_RUNNING, self.behaviour_previous, STATE_INACTIVE)
                self.mapping_run()

            else:  # If no attempts remaining on the current waypoint, goto the next waypoint
                self.set_waypoint_skipped(self.waypoints[self.waypoint_current])
                self.waypoint_current =+1
                self.go_to_next_waypoint()


    def mapping_reached_waypoint(self):
        '''
        Triggered when mapping reports that it has reached its assigned waypoint.
        '''
        self.set_waypoint_visited(self.waypoints[self.waypoint_current])
        if self.is_room_visited(self.room):
            self.room =+ 1
        
        self.go_to_next_waypoint()


    def mapping_override(self, behaviour):
        '''
        Called by one of the higher-priority nodes in order to take over control
        '''

        if behaviour == BEHAVIOUR_OBJECT_DETECTION:
            self.change_behaviour(BEHAVIOUR_MAPPING, STATE_ACTIVE_STOPPED, BEHAVIOUR_OBJECT_NAVIGATION, STATE_ACTIVE_RUNNING)
            self.mapping_control_publisher.publish(ACTION_STOP)
        else:
            self.change_behaviour(BEHAVIOUR_MAPPING, STATE_ACTIVE_STOPPED, behaviour, STATE_ACTIVE_RUNNING)
            self.mapping_control_publisher.publish(ACTION_PAUSE)


    def go_to_next_waypoint(self):
        '''
        Instructs mapping to go to the next waypoint if one is available. End the
        program if not.
        '''
        next_id = self.get_next_waypoint
        if next_id == -1:
            self.change_behaviour(BEHAVIOUR_MAPPING, STATE_INACTIVE, BEHAVIOUR_NONE, 0)
            self.mapping_control_publisher.publish(ACTION_STOP)
            self.object_finding_failed()
        else:
            self.waypoint_current = next_id
            self.change_behaviour(BEHAVIOUR_MAPPING, STATE_ACTIVE_RUNNING, self.behaviour_previous, STATE_INACTIVE)
            self.mapping_run()


    def skip_remaining_room_waypoints(self):
        '''
        Marks the waypoints in a room that come after the current waypoint as waypoints
        to visit if necessary, uot currently.
        '''
        for waypoint in self.waypoints:
            if waypoint["room"] == self.room and waypoint["id"] >= self.waypoint_current:
                self.set_waypoint_skipped(waypoint)


    def set_waypoint_skipped(self, waypoint):
        '''
        Marks a waypoint as needing re-visiting.
        '''
        waypoint["visited"] = False
        waypoint["re-visit"] = True
        waypoint["attempt"] = 0
        self.waypoints_skipped =+ 1


    def set_waypoint_visited(self, waypoint):
        '''
        Marks a waypoint as needing re-visiting.
        '''
        waypoint["visited"] = True
        waypoint["re-visit"] = False
        waypoint["attempt"] = 0
        self.waypoints_visited =+ 1


    def is_room_visited(self, room):
        '''
        Verifies that the current room has been completely visited
        '''
        if not self.get_next_waypoint == -1:
            if self.waypoints[self.waypoint_current]["room"] == room:
                return False
            else:
                return True
        else:
            return True


    def get_next_waypoint(self):
        '''
        Chooses the next waypoint.
        If there is a waypoint in the list that is unvisited and after the current
        one then that is chosen. If there isn't then the first waypoint with a
        re-visit flags is selected. If no waypoints are available in the re-vesit
        list and looping is enabled, the visit and re-visit flags are reset and the
        first waypoint is selected. If looping is disabled, no waypoint (-1) is returned. 
        '''
        for waypoint in self.waypoints:  # Return the next unvisited waypoint in the list
            if waypoint["visited"] == False and waypoint["re-visit"] == False and waypoint["id"] > self.waypoint_current:
                return waypoint["id"]
        
        # No unvisited waypoints left
        for waypoint in self.waypoints:  # Return the next re-visit waypoint in the list
            if waypoint["visited"] == False and waypoint["re-visit"] == True and waypoint["id"] > self.waypoint_current:
                return waypoint["id"]

        # No unvisited re-visits left after the current waypoint
        for waypoint in self.waypoints:  # Return the first available re-visit
            if waypoint["visited"] == False and waypoint["re-visit"] == True:
                return waypoint["id"]
        
        # If here, all waypoints have been visited and there are no re-visits left.
        if LOOPING:  # If looping, go back to the first
            for waypoint in self.waypoints:
                waypoint["attempt"] = 0
                waypoint["visited"] = False
                waypoint["re-visit"] = False
            return 0
        else:  # If not, admit defeat
            return -1
            
            
    def all_objects_found(self):
        '''
        Called when all objects are found.
        '''
        rospy.loginfo("All objects found")
        self.shutdown()


    def object_finding_failed(self):
        '''
        Called when there are still objects remaining but there are no more places to visit.
        '''
        rospy.loginfo("Failed to find all objects without repeating")
        self.shutdown()

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.shutting_down = True  # Terminates continuous loops
        rospy.loginfo("Stopping")
        rospy.sleep(1)


def main():
    '''
    Is initially called, inits the node and starts the class.
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
