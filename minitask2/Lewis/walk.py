#!/usr/bin/env python
import rospy
import tf

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from math import radians, pi as PI, sqrt, cos, sin, atan
from random import randint

FORWARD_SPEED = 0.125
TURN_SPEED = ((2 * PI) / 360) * 25  # 25 Deg/s
WALK_DISTANCE = 3.0
OFFSET_ERROR_LINEAR = 0.01  # Allowable offset from target is 0.01 meters
OFFSET_ERROR_ANGULAR = ((2 * PI) / 360) * 0.1   # Allowable offset from target is 0.1 degrees

class Walk():

    def __init__(self):
        rospy.init_node('Walk', anonymous = True)
        rospy.on_shutdown(self.shutdown)

        # For path plotting
        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        self.path = Path()
        self.plot_subscriber = rospy.Subscriber("odom", Odometry, self.plot_trajectory)

        # For movement
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.forward = Twist()
        self.forward.linear.x = FORWARD_SPEED
        self.turn_left = Twist()
        self.turn_left.angular.z = TURN_SPEED
        self.turn_right = Twist()
        self.turn_right.angular.z = (-TURN_SPEED)
        
        # For odometry
        self.curr_pose = {}
        self.curr_pose["theta"] = 0.0
        self.curr_pose["x"] = 0.0
        self.curr_pose["y"] = 0.0

        self.last_pose = None
        self.turning = True

        self.target = self.get_new_target()
        self.travel_angle = self.target["delta_theta"]

        rospy.spin()

    
    def random_walk(self, distance = 3):
        '''
        Waffle moves forward 3 meters, randomly chooses a direction and rotates to continue the random walk. 
        This behaviour is overriden by wall and obstacle detection.
        '''

    def drive_forward(self):
        self.vel_publisher.publish(self.forward)


    def drive_turn_left(self):
        self.vel_publisher.publish(self.turn_left)


    def drive_turn_right(self):
        self.vel_publisher.publish(self.turn_right)


    def drive_stop(self):
        self.vel_publisher.publish(Twist())


    def is_obstacle(self): # MOVE TO ANOTHER NODE
        '''
        Helper method to determine if an obstacle is present within 0.5m of the Waffle. 
        Publishes to the alert topic.
        '''


    def is_wall_right(self): # MOVE TO ANOTHER NODE
        '''
        Helper method used to detect if a wall is present on the right side. 
        Publishes to the alert topic.
        '''


    def odom_callback(self, odom):
        '''
        Get (x, y, theta) specification from odometry topic
        '''
        quart = [odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quart)

        self.curr_pose["theta"] = yaw
        self.curr_pose["x"] = odom.pose.pose.position.x
        self.curr_pose["y"] = odom.pose.pose.position.y

        self.plot_trajectory(odom)

        self.drive()

    
    def drive(self):
        if self.last_pose:
            if self.turning:

                if self.curr_pose["theta"] > abs(self.travel_angle):
                    self.turning = False
                    self.drive_stop()
                    print("[LOG]    Current angle: " + str(self.curr_pose["theta"]))
                    print("[LOG]    Driving forward: " + str(self.target["distance"]) + "m")
                else:
                    if self.travel_angle > 0:
                        self.drive_turn_left()
                    else:
                        self.drive_turn_right()
            else:
                distance_remaining = self.get_distance_remaining(self.curr_pose["x"], self.curr_pose["y"], self.target["x"], self.target["y"])
                print("[DEBUG]  Distance remaining: " + str(distance_remaining))
                if distance_remaining < OFFSET_ERROR_LINEAR:
                    print("[LOG]    Driving complete")
                    self.drive_stop()
                    self.turning = True
                    self.target = self.get_new_target
                else:
                    self.drive_forward()

        self.last_pose = self.curr_pose


    def get_new_target(self, travel_distance = WALK_DISTANCE):
        # set target position using polar coordinate maths
        random_angle = radians(randint(1, 360))
        target_pose = {}
        target_pose["theta"] = 0.0
        target_pose["delta_theta"] = 0.0
        target_pose["x"] = self.curr_pose["x"] + (travel_distance * cos(random_angle))
        target_pose["y"] = self.curr_pose["y"] + (travel_distance * sin(random_angle))
        print("[LOG]    Target position: (" + str(target_pose["x"]) + "," + str(target_pose["y"]) + ")")

        # convert to polar coordinates
        target_pose["distance"] = sqrt((target_pose["x"] ** 2) + (target_pose["y"] ** 2)) 
        target_pose["theta"] = self.euclidian_to_polar(target_pose["x"], target_pose["y"])
        print("[LOG]    Distance: " + str(target_pose["distance"]) + ", Angle: " + str(target_pose["theta"]))

        # determine angle to rotate by to match target angle
        if self.curr_pose["theta"] > target_pose["theta"]:
            target_pose["delta_theta"] = -(self.curr_pose["theta"] - target_pose["theta"])
        else:
            target_pose["delta_theta"] =  (target_pose["theta"] - self.curr_pose["theta"])
        print("[LOG]    Current angle: " + str(self.curr_pose["theta"]))
        
        return target_pose


    def euclidian_to_polar(self, x, y):
        angle = atan(y/x)

        if  (x > 0 and y > 0):
            return angle

        elif (x < 0 and y > 0):
            return (angle + PI)

        elif (x < 0 and y < 0): 
            return (0.0-(PI - angle))
            
        elif (x > 0 and y < 0):
            return angle
        
        return 0


    def get_distance_remaining(self, current_x, current_y, target_x, target_y):
        if current_x > target_x:
            if current_y > target_y:
                return sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)
            else:
                return sqrt((current_x - target_x) ** 2 + (target_y - current_y) ** 2)
        else:
            if current_y > target_y:
                return sqrt((target_x - current_x) ** 2 + (current_y - target_y) ** 2)
            else:
                return sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

    
    def get_distance_remaining_1d(self, current_x, target_x):
        if current_x > target_x:
            return sqrt((current_x - target_x) ** 2)
        else:
            return sqrt((target_x - current_x) ** 2)



    def plot_trajectory(self, odom):
        '''
        Publishes the odometry data to /path so rviz can plot it.
        '''
        p = PoseStamped()
        p.header = odom.header
        p.pose = odom.pose.pose

        self.path.header = odom.header
        self.path.poses.append(p)
        self.path_publisher.publish(self.path)


    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.vel_publisher.publish(Twist())
        print("[EXIT]   Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Walk()
    except rospy.ROSInterruptException:
        pass