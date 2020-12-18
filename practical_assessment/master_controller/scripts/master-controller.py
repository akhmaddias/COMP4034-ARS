#!/usr/bin/env python
'''
The root controller class for the Turtlebot3 Waffle.
This implements an object search behaviour that enables the robot to search
for predefined objects visible in the robot's camera. The controller selects
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
__author__ = "Lewis C Brand"

import threading
import rospy

from master_controller.msg import DetectedObject
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Pose


# Route designed for maximum visual coverage with non-overlapping viewing areas
WAYPOINTS = [
    #(-1.3, 4.05, 0, 0),  # Reverse robot waypoint
    (-1.2, -0.90, 0, 0),
    (0.0, 0.5, 0, 1),
    (3.3, 0.8, 0, 2),
    (5.5, 2.9, 0, 2),
    (5.9, -3.0, 0, 2),
    (5.0, -3.0, 0, 2),  # Reverse robot waypoint
    (0.8, -3.4, 0, 1),
    (0.0, 4.0, 0, 3),
    (3.5, 4.0, 0, 4),
    (1.5, 2.5, 0, 4)
]

OBJECTS = [
    "Fire Hydrant",
    "Green Box",
    "Blue Postbox",
    "Black and White 5"
]

#  Overall parameters
MAX_REATTEMPTS_PER_WAYPOINT = 2
MAX_REVISITS_PER_WAYPOINT = 1 
LOOPING = False
ENABLE_SKIP_ROOMS = False

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
BEHAVIOUR_COLLISION_AVOIDANCE = 2
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
NOT_IN_RECOVERY = 0
FAILED = 5


class Controller():
    '''
    The controller class.
    '''

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.shutting_down = False
        self.behaviour_change_lock = threading.Lock()
        self.out_of_controlling_state_lock = threading.Lock()

        rospy.loginfo("Initialising state model")

        #  State variables
        self.state_mapping = STATE_INACTIVE
        self.state_object_detection = STATE_ACTIVE_STOPPED
        self.state_object_navigation = STATE_INACTIVE
        self.state_collision_avoidance = STATE_INACTIVE
        self.state_recovery = STATE_INACTIVE

        #  Behavioural
        self.behaviour_current = BEHAVIOUR_NONE
        self.behaviour_previous = BEHAVIOUR_NONE

        #  Navigation
        self.waypoints = []
        self.waypoints_visited = 0
        self.waypoints_discarded = 0
        self.waypoints_skipped = 0
        self.waypoint_current = -1
        self.room = 0
        self.objects_found = 0

        #  Goal tracking
        self.objects = []
        self.object_current = -1
        self.objects_found = 0
        self.objects_total = len(OBJECTS)
        rospy.loginfo("Task has {} objects to find".format(self.objects_total))
        self.objects_mapping_aware = 0

        self.init_waypoints()
        self.init_objects()

        self.init_pub_subs()
        rospy.loginfo("Initialised")

    def controller_start(self):
        '''
        First method to run. Gets the first waypoint and prepares the state change.
        '''
        rospy.loginfo("Controller starting...")
        rospy.sleep(1)
        rospy.loginfo("Controller started")
        next_id = self.get_next_waypoint()
        if next_id == (-1):
            rospy.logwarn("No waypoints to visit!")
            self.object_finding_failed()
        else:
            rospy.loginfo("Going to waypoint {}".format(next_id))
            self.waypoint_current = next_id
            self.behaviour_previous = BEHAVIOUR_MAPPING
            self.change_behaviour(
                BEHAVIOUR_NONE, 0, BEHAVIOUR_MAPPING, STATE_ACTIVE_RUNNING)
            self.mapping_run()

    def init_waypoints(self):
        '''
        Initialises waypoint array.
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
            waypoint_details["re-visit_count"] = 0
            waypoint_details["visited"] = False
            waypoint_details["re-visit"] = False
            waypoint_details["discarded"] = False

            self.waypoints.append(waypoint_details)
            i += 1

    def init_objects(self):
        '''
        Initialises objects array.
        '''
        rospy.loginfo("Initialising objects")
        i = 0
        for detectable_object in OBJECTS:
            object_details = {}
            object_details["id"] = i
            object_details["x"] = 0.0
            object_details["y"] = 0.0
            object_details["size_x"] = 0.0
            object_details["size_y"] = 0.0
            object_details["z"] = 0.0
            object_details["name"] = detectable_object
            object_details["visited"] = False
            object_details["visiting"] = False

            self.objects.append(object_details)
            i += 1

    def init_pub_subs(self):
        '''
        Registers all necessary publishers and subscribers in order to
        communicate with the appropriate nodes.
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
        self.object_detection_subscriber = rospy.Subscriber(
            '/objects', DetectedObject, self.object_detection_callback)

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
        self.object_navigation_detected_publisher = rospy.Publisher(
            'object_navigation_detected', DetectedObject, queue_size=10)
        self.object_navigation_control_publisher = rospy.Publisher(
            'object_control', Int32, queue_size=10)
        self.object_pose_subscriber = rospy.Subscriber(
            'object_position', Pose, self.object_position_callback)
        self.object_control_subscriber = rospy.Subscriber(
            'object_control', Int32, self.object_control_callback)

    def mapping_control_callback(self, msg):
        if msg.data == REACHED_WAYPOINT:
            self.mapping_reached_waypoint()
        elif msg.data == FAILED:
            self.mapping_failed()
        elif msg.data != ACTION_START and msg.data != ACTION_PAUSE and msg.data != ACTION_STOP:
            rospy.logwarn("Unhandled Mapping Control callback message!")
            rospy.logwarn("{}".format(msg.data))

    def object_detection_type_callback(self, msg):
        rospy.logwarn("Unhandled Object Detection Type callback message!")

    def object_detection_pose_callback(self, msg):
        rospy.logwarn("Unhandled Object Detection Pose callback message!")

    def object_detection_callback(self, msg):
        self.object_detection_detected(
            msg.object_name, msg.x, msg.y, msg.size_x, msg.size_y)

    def collision_control_callback(self, msg):
        if msg.data == COLLISION_TO_AVOID:
            self.collision_avoidance_run()
        elif msg.data == NO_COLLISION_TO_AVOID:
            self.collision_avoidance_finish()
        elif msg.data != ACTION_STOP:
            rospy.logwarn("Unhandled Collision Avoidance callback message!")

    def recovery_control_callback(self, msg):
        if msg.data == IN_RECOVERY:
            self.recovery_behaviour_run()
        elif msg.data == NOT_IN_RECOVERY:
            self.recovery_behaviour_finished()
        elif msg.data != ACTION_STOP:
            rospy.logwarn("Unhandled Collision Avoidance callback message!")

    def object_position_callback(self, msg):
        rospy.logwarn("Unhandled Object Navigation Pose callback message!")

    def object_control_callback(self, msg):
        if msg.data == REACHED_OBJECT:
            self.object_navigation_reached_object(0.0, 0.0)
        elif msg.data == FAILED:
            self.object_navigation_failed()
        elif msg.data != ACTION_START and msg.data != ACTION_PAUSE and msg.data != ACTION_STOP:
            rospy.logwarn(
                "Unhandled Object Navigation Control callback message")

    def mapping_send_start(self):
        self.mapping_control_publisher.publish(ACTION_START)
        rospy.loginfo("Sent START to Mapping")

    def mapping_send_stop(self):
        self.mapping_control_publisher.publish(ACTION_STOP)
        rospy.loginfo("Sent STOP to Mapping")

    def mapping_send_pause(self):
        self.mapping_control_publisher.publish(ACTION_PAUSE)
        rospy.loginfo("Sent PAUSE to Mapping")

    def mapping_send_coordinates(self, waypoint):
        '''
        Translates and then sends a waypoint to the mapping node.
        '''
        pose = Pose()
        pose.position.x = waypoint["x"]
        pose.position.y = waypoint["y"]
        pose.position.z = waypoint["z"]
        self.mapping_pose_publisher.publish(pose)
        rospy.loginfo("Sent coordinates to Mapping.")

    def object_navigation_send_start(self):
        self.object_navigation_control_publisher.publish(ACTION_START)
        rospy.loginfo("Sent START to Object Navigation")

    def object_navigation_send_pause(self):
        self.object_navigation_control_publisher.publish(ACTION_PAUSE)
        rospy.loginfo("Sent PAUSE to Object Navigation")

    def object_navigation_send_stop(self):
        self.object_navigation_control_publisher.publish(ACTION_STOP)
        rospy.loginfo("Sent STOP to Object Navigation")

    def object_navigation_send_detected(self):
        msg = DetectedObject()
        msg.x = self.objects[self.object_current]["x"]
        msg.y = self.objects[self.object_current]["y"]
        msg.size_x = self.objects[self.object_current]["size_x"]
        msg.size_y = self.objects[self.object_current]["size_y"]
        msg.object_name = self.objects[self.object_current]["name"]
        self.object_navigation_detected_publisher.publish(msg)
        #rospy.loginfo("Sent object details to Object Navigation")

    def object_navigation_send_object_coordinates(self):
        pose = Pose()
        pose.position.x = self.objects[self.object_current]["x"]
        pose.position.y = self.objects[self.object_current]["y"]
        pose.position.z = self.objects[self.object_current]["z"]
        self.object_navigation_pose_publisher.publish(pose)
        rospy.loginfo("Sent coordinates to Object Navigation")

    def object_navigation_send_object_type(self):
        self.objects[self.object_current]["visiting"] = True
        self.object_navigation_type_publisher.publish(
            self.objects[self.object_current]["name"])
        # rospy.loginfo("Sent object type to Object Navigation")

    def change_behaviour(
            self, current_behaviour,
            current_behaviour_new_state,
            new_behaviour,
            new_behaviour_new_state):
        '''
        Signals a change in current behaviour and changes states appropriately.
        Locks the thread so this happens as an atomic block. This prevents two behaviours
        changing state at once and creating a rece condition.
        '''
        with self.behaviour_change_lock:
            rospy.loginfo("Behaviour changed from " +
                          self.get_behaviour_name(current_behaviour) +
                          " to " +
                          self.get_behaviour_name(new_behaviour))

            self.behaviour_previous = current_behaviour
            self.behaviour_current = new_behaviour

            self.change_behaviour_state(
                current_behaviour, current_behaviour_new_state)
            self.change_behaviour_state(new_behaviour, new_behaviour_new_state)

    def change_behaviour_state(self, behaviour, new_state):
        '''
        Changes the current state of a behaviour.
        '''
        past_state = 0
        if behaviour == BEHAVIOUR_MAPPING:
            past_state = self.state_mapping
            self.state_mapping = new_state
        elif behaviour == BEHAVIOUR_OBJECT_NAVIGATION:
            past_state = self.state_object_navigation
            self.state_object_navigation = new_state
        elif behaviour == BEHAVIOUR_RECOVERY:
            past_state = self.state_recovery
            self.state_recovery = new_state
        elif behaviour == BEHAVIOUR_COLLISION_AVOIDANCE:
            past_state = self.state_recovery
            self.state_collision_avoidance = new_state

        rospy.loginfo("{} changed state from {} to {}".format(
            self.get_behaviour_name(behaviour),
            self.get_state_name(past_state),
            self.get_state_name(new_state)))

    def get_behaviour_name(self, behaviour):
        '''
        Given a behaviour, returns a string of the name of the behaviour.
        '''
        behaviour_string = ""
        if behaviour == BEHAVIOUR_MAPPING:
            behaviour_string = "Mapping"
        elif behaviour == BEHAVIOUR_OBJECT_DETECTION:
            behaviour_string = "Object detection"
        elif behaviour == BEHAVIOUR_OBJECT_NAVIGATION:
            behaviour_string = "Object navigation"
        elif behaviour == BEHAVIOUR_RECOVERY:
            behaviour_string = "Recovery behaviour"
        elif behaviour == BEHAVIOUR_COLLISION_AVOIDANCE:
            behaviour_string = "Collision avoidance"
        elif behaviour == BEHAVIOUR_NONE:
            behaviour_string = "None"
        return behaviour_string

    def get_state_name(self, state):
        '''
        Given a state, returns a string of the name of the state.
        '''
        state_string = ""
        if state == STATE_ACTIVE_RUNNING:
            state_string = "ACTIVE RUNNING"
        elif state == STATE_ACTIVE_PAUSED:
            state_string = "ACTIVE PAUSED"
        elif state == STATE_ACTIVE_STOPPED:
            state_string = "ACTIVE STOPPED"
        elif state == STATE_IGNORED:
            state_string = "STATE IGNORED"
        elif state == STATE_INACTIVE:
            state_string = "STATE INACTIVE"
        else:
            state_string = "UNKNOWN STATE"
        return state_string

    def return_to_previous_behaviour(self):
        '''
        Signals a return from the current behaviour to the behaviour previous to it
        and changes the states appropriately.
        '''
        with self.behaviour_change_lock:
            past_behaviour = self.behaviour_previous
            current_behaviour = self.behaviour_current
            self.return_to_behaviour(current_behaviour, past_behaviour)

    def return_to_behaviour(self, current_behaviour, past_behaviour):
        '''
        Returns to a behaviour. Should not be used directly.
        '''
        rospy.loginfo("Behaviour returning from " +
                      self.get_behaviour_name(current_behaviour) +
                      " to " +
                      self.get_behaviour_name(past_behaviour))

        self.behaviour_previous = current_behaviour
        self.behaviour_current = past_behaviour
        
        if past_behaviour == BEHAVIOUR_MAPPING:
            past_behaviour_past_state = self.state_mapping
        elif past_behaviour == BEHAVIOUR_OBJECT_NAVIGATION:
            past_behaviour_past_state = self.state_object_navigation

        self.change_behaviour_state(current_behaviour, STATE_INACTIVE)
        self.change_behaviour_state(past_behaviour, STATE_ACTIVE_RUNNING)

        if past_behaviour == BEHAVIOUR_MAPPING:
            self.mapping_resume(past_behaviour_past_state)

        elif past_behaviour == BEHAVIOUR_OBJECT_NAVIGATION:
            self.object_navigation_resume()

    def mapping_run(self):
        '''
        Method used to instruct the mapping node to perform a task.
        '''
        self.mapping_send_coordinates(self.waypoints[self.waypoint_current])
        self.mapping_send_start()

    def mapping_resume(self, previous_state):
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
        if previous_state == STATE_ACTIVE_PAUSED:
            self.mapping_send_start()

        elif previous_state == STATE_ACTIVE_STOPPED:
            if self.objects_found > self.objects_mapping_aware:  # New object found
                if self.objects_found >= self.objects_total:  # If there are no more objects left
                    self.mapping_send_stop()
                    # self.change_behaviour(
                    #     BEHAVIOUR_MAPPING, STATE_INACTIVE, BEHAVIOUR_NONE, 0)
                    self.all_objects_found()

                else:  # If resuming, objects found but still more to go
                    if ENABLE_SKIP_ROOMS:
                        self.skip_remaining_room_waypoints()  # Go to the next room
                    self.go_to_next_waypoint()

            # If no objects found
            else:
                self.reattempt_waypoint_or_skip()

    def mapping_reached_waypoint(self):
        '''
        Triggered when mapping reports that it has reached its assigned waypoint.
        '''
        self.set_waypoint_visited(self.waypoints[self.waypoint_current])
        self.go_to_next_waypoint()

    def mapping_override(self, behaviour):
        '''
        Called by one of the higher-priority nodes in order to take over control.
        '''
        if behaviour == BEHAVIOUR_OBJECT_DETECTION:
            self.change_behaviour(BEHAVIOUR_MAPPING,
                                  STATE_ACTIVE_STOPPED,
                                  BEHAVIOUR_OBJECT_DETECTION,
                                  STATE_ACTIVE_RUNNING)
            self.mapping_send_stop()
        else:
            self.change_behaviour(BEHAVIOUR_MAPPING,
                                  STATE_ACTIVE_PAUSED,
                                  behaviour,
                                  STATE_ACTIVE_RUNNING)
            self.mapping_send_pause()
    
    def mapping_failed(self):
        '''
        Called when mapping has failed.
        '''
        if self.state_mapping == STATE_ACTIVE_RUNNING:    
            rospy.logerr("Mapping failed to reach waypoint {}!".format(self.waypoint_current))
            self.reattempt_waypoint_or_skip()

    def object_detection_detected(self, object_name, coordinate_x, coordinate_y, size_x, size_y):
        '''
        Called when an object is detected.
        If mapping is currently active (and therefore not overridden), and the object that has been
        detected is new and currently not being visited, this method will override mapping and start
        object navigation. Uses a thread lock to prevent overriding behaviours executing whilst
        there are no motion controlling behaviours running.
        '''
        self.out_of_controlling_state_lock.acquire()
        rospy.logdebug("Out of controlling state lock acquired")

        object_id = self.get_object_id(object_name)

        if object_id != -1:
            self.object_current = object_id
            self.objects[self.object_current]["x"] = coordinate_x
            self.objects[self.object_current]["y"] = coordinate_y
            self.objects[self.object_current]["size_x"] = size_x
            self.objects[self.object_current]["size_y"] = size_y
            visited = self.objects[self.object_current]["visited"]
            visiting = self.objects[self.object_current]["visiting"]
            if self.state_mapping == STATE_ACTIVE_RUNNING and not visited and not visiting:
                rospy.loginfo("### Identified {} ###".format(self.objects[self.object_current]["name"]))
                self.objects[self.object_current]["visiting"] = True
                self.mapping_override(BEHAVIOUR_OBJECT_DETECTION)
                self.change_behaviour(BEHAVIOUR_OBJECT_DETECTION,
                                      STATE_INACTIVE,
                                      BEHAVIOUR_OBJECT_NAVIGATION,
                                      STATE_ACTIVE_RUNNING)
                self.object_navigation_run()
            elif self.state_object_navigation == STATE_ACTIVE_RUNNING  and visiting and not visited:
                self.object_navigation_send_detected()

        else:
            rospy.logerr("Unrecognised object presented!")

        self.out_of_controlling_state_lock.release()
        rospy.logdebug("Out of controlling state lock released")

    def collision_avoidance_run(self):
        '''
        Triggered when obstacle avoidance detects something and needs to take over.
        Acquires the out_of_controlling_state_lock to ensure that avoidance_behaviour
        will override a controlling behaviour.
        '''
        self.out_of_controlling_state_lock.acquire()
        rospy.logdebug("Out of controlling state lock acquired")
        rospy.logwarn("Collision alert")
        if self.state_object_navigation == STATE_ACTIVE_RUNNING:
            self.object_navigation_override()
        elif self.state_mapping == STATE_ACTIVE_RUNNING:
            self.mapping_override(BEHAVIOUR_COLLISION_AVOIDANCE)
        self.out_of_controlling_state_lock.release()
        rospy.logdebug("Out of controlling state lock released")

    def collision_avoidance_finish(self):
        '''
        Triggered when obstacle avoidance is no longer detecting anything.
        Acquires the out_of_controlling_state_lock to ensure that avoidance_behaviour
        will override a controlling behaviour.
        '''
        self.out_of_controlling_state_lock.acquire()
        rospy.logdebug("Out of controlling state lock acquired")
        if self.state_collision_avoidance == STATE_ACTIVE_RUNNING:
            self.return_to_previous_behaviour()
        else:
            rospy.logerr("Collision avoidance attempted to change into existing state!")
        self.out_of_controlling_state_lock.release()
        rospy.logdebug("Out of controlling state lock released")

    def recovery_behaviour_run(self):
        '''
        Triggered when recovery behaviour is initiated and needs to take over.
        Acquires the out_of_controlling_state_lock to ensure that avoidance_behaviour
        will override a controlling behaviour.
        '''

        self.out_of_controlling_state_lock.acquire()
        rospy.logdebug("Out of controlling state lock acquired")

        if self.state_mapping == STATE_ACTIVE_RUNNING:
            self.mapping_override(BEHAVIOUR_RECOVERY)
        elif self.state_object_navigation == STATE_ACTIVE_RUNNING:
            self.object_navigation_override()

        self.out_of_controlling_state_lock.release()
        rospy.logdebug("Out of controlling state lock released")

    def recovery_behaviour_finished(self):
        '''
        Triggered when recovery behaviour is no longer detecting anything.
        Acquires the out_of_controlling_state_lock to ensure that avoidance_behaviour
        will override a controlling behaviour.
        '''
        self.out_of_controlling_state_lock.acquire()
        rospy.logdebug("Out of controlling state lock acquired")

        self.return_to_previous_behaviour()

        self.out_of_controlling_state_lock.release()
        rospy.logdebug("Out of controlling state lock released")

    def object_navigation_run(self):
        '''
        Called there is an object to navigate to. Details of the object should be supplied
        by setting the current_object.
        '''
        self.object_navigation_send_detected()

        self.object_navigation_send_start()

    def object_navigation_resume(self):
        '''
        Resumes navigation to an object from a paused state.
        '''
        self.object_navigation_send_start()

    def object_navigation_reached_object(self, x_coordinate, y_coordinate):
        '''
        Called when the object has been reached.
        This returns to mapping once complete.
        '''
        rospy.loginfo("Reached message received")
        if not self.objects[self.object_current]["visited"]: 
            rospy.loginfo("### Object reached! - {} ###".format(self.objects[self.object_current]["name"]))
            self.objects[self.object_current]["visited"] = True
            self.objects[self.object_current]["visiting"] = False
            self.objects[self.object_current]["x"] = x_coordinate
            self.objects[self.object_current]["y"] = y_coordinate
            self.objects[self.object_current]["z"] = 0.0
            self.objects_found += 1
            rospy.loginfo("Found {} objects".format(self.objects_found))
            self.object_navigation_send_stop()

            self.return_to_behaviour(self.behaviour_current, BEHAVIOUR_MAPPING)

    def object_navigation_failed(self):
        '''
        Called when object navigation has failed.
        '''
        if self.state_object_navigation == STATE_ACTIVE_RUNNING:    
            rospy.logerr("Navigation to {} failed!".format( self.objects[self.object_current]["name"]))
            self.objects[self.object_current]["visiting"] = False
            self.objects[self.object_current]["visited"] = False
            self.object_navigation_send_stop()

            self.return_to_behaviour(self.behaviour_current, BEHAVIOUR_MAPPING)


    def object_navigation_override(self, behaviour):
        '''
        Method to override navigation in case of an exceptional circumstance.
        '''
        self.object_navigation_send_pause()
        self.mapping_send_stop()

    def get_object_id(self, object_name):
        '''
        Returns an object id based on the object name.
        '''
        for object_instance in self.objects:
            if object_instance["name"] == object_name:
                return object_instance["id"]
        return -1

    def go_to_next_waypoint(self):
        '''
        Instructs mapping to go to the next waypoint if one is available. End the
        program if not.
        '''
        next_id = self.get_next_waypoint()
        if next_id == (-1):
            rospy.logwarn("No waypoints to visit!")
            self.change_behaviour(
                BEHAVIOUR_MAPPING, STATE_INACTIVE, self.behaviour_previous, 0)
            self.mapping_send_stop()
            self.object_finding_failed()
        else:
            rospy.loginfo("Going to waypoint {}".format(next_id))
            self.waypoint_current = next_id
            self.room = self.waypoints[next_id]["room"]
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
        waypoint["re-visit_count"] += 1

        if waypoint["re-visit_count"] >= MAX_REVISITS_PER_WAYPOINT:
            self.set_waypoint_discarded(waypoint)
            rospy.logwarn("Skipped waypoint {} has exceeded re-visit count. Waypoint will be discarded.".format(waypoint["id"]))
        else:
            waypoint["visited"] = False
            waypoint["re-visit"] = True
            waypoint["attempt"] = 0
            self.waypoints_skipped += 1
            rospy.logwarn("Skipped waypoint {}".format(waypoint["id"]))

    def set_waypoint_visited(self, waypoint):
        '''
        Marks a waypoint as visited.
        '''
        waypoint["visited"] = True
        waypoint["re-visit"] = False
        waypoint["attempt"] = 0
        self.waypoints_visited += 1
        rospy.loginfo("Reached waypoint {}".format(waypoint["id"]))

    def set_waypoint_discarded(self, waypoint):
        '''
        Marks a waypoint as visited.
        '''
        waypoint["visited"] = False
        waypoint["re-visit"] = False
        waypoint["attempt"] = 0
        waypoint["discarded"] = True
        self.waypoints_discarded += 1
        rospy.loginfo("Waypoint {} discarded.".format(waypoint["id"]))

    def is_room_visited(self, room):
        '''
        Verifies that the current room has been completely visited.
        '''
        if self.get_next_waypoint() != -1:
            next_id = self.get_next_waypoint()
            if self.waypoints[next_id]["room"] == room:
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
            if not waypoint["discarded"] and not waypoint["visited"] and not waypoint["re-visit"] and waypoint["id"] > self.waypoint_current:
                return waypoint["id"]

        # No unvisited waypoints left
        for waypoint in self.waypoints:  # Return the next re-visit waypoint in the list
            if not waypoint["discarded"] and not waypoint["visited"] and waypoint["re-visit"] and waypoint["id"] > self.waypoint_current:
                return waypoint["id"]

        # No unvisited re-visits left after the current waypoint
        for waypoint in self.waypoints:  # Return the first available re-visit
            if not waypoint["discarded"] and not waypoint["visited"] and waypoint["re-visit"]:
                return waypoint["id"]

        # If here, all waypoints have been visited and there are no re-visits left.
        if LOOPING:  # If looping, go back to the first
            for waypoint in self.waypoints:
                waypoint["attempt"] = 0
                waypoint["re-visit_count"] = 0
                waypoint["visited"] = False
                waypoint["re-visit"] = False
            return 0
        else:  # If not, admit defeat
            return -1

    def reattempt_waypoint_or_skip(self):
        self.waypoints[self.waypoint_current]["attempt"] += 1
        if self.waypoints[self.waypoint_current]["attempt"] <= MAX_REATTEMPTS_PER_WAYPOINT:
            # If the waypoint still has attempts left
            rospy.logwarn("Waypoint {} attempt number {} failed. Will re-attempt.".format(self.waypoint_current, 
                self.waypoints[self.waypoint_current]["attempt"]))
            self.mapping_run()
        else:  
            # If no attempts remaining on the current waypoint, goto the next waypoint
            rospy.logwarn("Waypoint {} attempt number {} failed.".format(self.waypoint_current, 
                self.waypoints[self.waypoint_current]["attempt"]))
            self.set_waypoint_skipped(
                self.waypoints[self.waypoint_current])
            self.go_to_next_waypoint()

    def all_objects_found(self):
        '''
        Called when all objects are found.
        '''
        rospy.loginfo("### ALL OBJECTS FOUND! ###")
        self.shutdown()

    def object_finding_failed(self):
        '''
        Called when there are still objects remaining but there are no more places to visit.
        '''
        rospy.logfatal("Failed to find all objects without repeating")
        self.shutdown()

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.shutting_down = True  # Terminates continuous loops
        rospy.logwarn("Stopping - shutdown")
        self.mapping_send_stop()
        self.object_navigation_send_stop()
        rospy.signal_shutdown("Stopping - shutdown")


def main():
    '''
    Is initially called, inits the node and starts the class.
    '''

    rospy.init_node('master_controller', anonymous=True)
    Controller().controller_start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Stopping - main")
        rospy.signal_shutdown("Stopping - main")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
