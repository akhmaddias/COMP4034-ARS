#!/usr/bin/env python
'''
Will beacon a Turtlebot3 Waffle toward green objects or wander around if not.
'''

from random import uniform
from math import sqrt, pi as PI

import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from cv2 import cv2
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations

FORWARD_SPEED = 0.2
TURN_SPEED = ((2 * PI) / 360) * 25  # 25 Deg/s
WALK_DISTANCE = 3.0
MIN_DISTANCE = 0.3

class Follower():  # pylint: disable=too-many-instance-attributes
    '''
    Will beacon a Turtlebot3 Waffle toward green objects or wander around if not.
    '''

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.twist = Twist()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # For path plotting
        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        self.path = Path()
        self.plot_subscriber = rospy.Subscriber("odom", Odometry, self.plot_trajectory)

        # For obstacle avoidance
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.obstacle = True
        self.direction = 0
        self.direction_set = False

        # For odometry and position tracking
        self.curr_pose = {}
        self.curr_pose["theta"] = 0.0
        self.curr_pose["x"] = 0.0
        self.curr_pose["y"] = 0.0
        self.last_pose = {}
        self.last_pose["theta"] = 0.0
        self.last_pose["x"] = 0.0
        self.last_pose["y"] = 0.0
        self.turning = False
        self.travel_angle = 0.0
        self.object_detected = False
        self.objects = 0
        rospy.loginfo("No object detected")

    def drive_turn_left(self):
        '''
        Turns left
        '''
        turn_left = Twist()
        turn_left.linear.x = 0
        turn_left.angular.z = TURN_SPEED
        self.vel_pub.publish(turn_left)

    def drive_turn_right(self):
        '''
        Turns right
        '''
        turn_right = Twist()
        turn_right.linear.x = 0
        turn_right.angular.z = -TURN_SPEED
        self.vel_pub.publish(turn_right)

    def drive_forward(self):
        '''
        Drives forwards
        '''
        forward = Twist()
        forward.linear.x = FORWARD_SPEED
        forward.angular.z = 0
        self.vel_pub.publish(forward)

    def drive_stop(self):
        '''
        Stops driving
        '''
        stop = Twist()
        stop.linear.x = 0
        stop.angular.z = -TURN_SPEED
        self.vel_pub.publish(stop)

    def image_callback(self, data):
        '''
        Handles the camera data.
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,'bgr8')
        except CvBridgeError as error:  # catch conversion errors
            print(error)

        cv_image_resized = cv2.resize(cv_image, (cv_image.shape[1]/4,cv_image.shape[0]/4))
        mask = self.get_mask(cv_image_resized)
        objects = self.get_centroids(mask)

        self.draw_centroids(objects, cv_image_resized)

        self.drive(objects, cv_image_resized.shape[1])

        cv2.waitKey(3)

    def drive(self, objects, image_width):
        '''
        Driver method.
        Selects beaconing if green objects are detected.
        If no green objects are detected, random walk is used instead.
        '''
        if not self.obstacle:
            if len(objects) > 0:
                if not self.object_detected:
                    self.object_detected = True
                    rospy.loginfo("Beaconing to selected object")
                self.drive_beacon(objects, image_width)
            else :
                if self.object_detected:
                    self.object_detected = False
                    rospy.loginfo("No object detected")
                self.drive_random_walk()

    def drive_beacon(self, objects, image_width):
        '''
        Proportional controller to beacon towards a target object.
        Assumes the first object in the array is the target.
        '''
        err = objects[0][0] - image_width / 2
        self.twist.linear.x = 0.2
        self.twist.angular.z = - float(err) / 100
        self.vel_pub.publish(self.twist)

    def drive_random_walk(self):
        '''
        Will move forward a predefined distance and then rotate by a random angle
        '''
        if self.turning:
            if (self.travel_angle < 0 and self.curr_pose["theta"] > self.travel_angle) \
            or (self.travel_angle >= 0 and self.curr_pose["theta"] < self.travel_angle):
                if self.travel_angle > 0:
                    self.drive_turn_left()
                else:
                    self.drive_turn_right()
            else:
                rospy.loginfo("Driving without objects")
                self.drive_stop()
        else:
            progress = sqrt((self.last_pose["x"] - self.curr_pose["x"])** 2 \
            + (self.last_pose["y"] - self.curr_pose["y"])** 2)
            if progress <= WALK_DISTANCE:
                self.drive_forward()
                self.turning = False
            else:
                rospy.loginfo("Turning")
                self.drive_stop()
                self.travel_angle = uniform(-PI, PI)
                self.set_location_ref()
                self.set_angle()
                self.turning = True

    def scan_callback(self, scan):
        '''
        Handles the scanner data and determines the correct action
        '''
        min_val_angle = 0
        min_val = 50
        for i in range(300, 359):
            if scan.ranges[i] < min_val:
                min_val = scan.ranges[i]
                min_val_angle = i
        for i in range(0, 60):
            if scan.ranges[i] < min_val:
                min_val = scan.ranges[i]
                min_val_angle = i

        if min_val < MIN_DISTANCE:
            if not self.obstacle:
                rospy.loginfo("Obstacle detected")
                self.drive_stop()
                self.obstacle = True
                self.direction_set = False
            self.obstacle_avoidance(min_val_angle)
        else:
            if self.obstacle:
                rospy.loginfo("No obstacle detected")
                self.obstacle = False
                self.direction = 0
                self.direction_set = False
                self.drive_stop()

    def obstacle_avoidance(self, min_val_angle):
        '''
        Will identify the obstacle in the path of the Waffle and attempt to avoid it.
        '''
        if not self.direction_set:
            rospy.loginfo("Obstacle avoidance")
            if min_val_angle < 90:
                self.direction = 2
                self.direction_set = True
            else:
                self.direction = 1
                self.direction_set = True
        else:
            if self.direction == 1:
                self.drive_turn_left()
            else:
                self.drive_turn_right()

    def draw_centroids(self, objects, image):
        '''
        Draws a magenta circle on the selected object and a red circle on the other objects.
        Assumes the first object in the array is the target.
        '''
        for i, coordinates in enumerate(objects):
            if i == 0:
                cv2.circle(image, coordinates, 5, (255, 0, 255), 2)
            else:
                cv2.circle(image, coordinates, 5, (0, 0, 255), 2)
        cv2.imshow("targets", image)

    def get_centroids(self, mask):
        '''
        Returns an array of centroids based on the number of masked objects
        '''
        contours, _ = cv2.findContours(mask.copy(),
                                        cv2.RETR_TREE,
                                        cv2.CHAIN_APPROX_SIMPLE,
                                        offset=(0, 0))  # Finds the outlines of each object
        centroids = []
        for contour in contours:  # Creates an array of the centres of each object
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                center_x = int(moments['m10']/moments['m00'])
                center_y = int(moments['m01']/moments['m00'])
                centroids.append((center_x, center_y))

        if len(contours) != self.objects:
            self.objects = len(contours)
            rospy.loginfo("Objects: " + str(self.objects))

        return centroids

    def get_mask(self, image):
        '''
        Returns an image mask based on the colour green
        '''
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low_green = np.array([0, 100, 0])
        high_green = np.array([100, 255, 255])
        mask = cv2.inRange(image_hsv, low_green, high_green)
        return mask

    def set_angle(self):
        '''
        Sets a random angle
        '''
        self.travel_angle = uniform(-PI, PI)
        rospy.loginfo("Travel angle: %f", self.travel_angle)

    def set_location_ref(self):
        '''
        Records the current location x, y and theta as the last location
        '''
        rospy.loginfo("Recording last position")
        self.last_pose["theta"] = self.curr_pose["theta"]
        self.last_pose["x"] = self.curr_pose["x"]
        self.last_pose["y"] = self.curr_pose["y"]
        rospy.loginfo("Theta: %f, x: %f, y: %f",
                       self.last_pose["theta"],
                       self.last_pose["x"],
                       self.last_pose["y"])

    def odom_callback(self, odom):
        '''
        Get (x, y, theta) specification from odometry topic
        '''
        quart = [odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w]

        (_, _, yaw) = transformations.euler_from_quaternion(quart)

        self.curr_pose["theta"] = yaw
        self.curr_pose["x"] = odom.pose.pose.position.x
        self.curr_pose["y"] = odom.pose.pose.position.y

        self.plot_trajectory(odom)

    def plot_trajectory(self, odom):
        '''
        Publishes the odometry data to /path so rviz can plot it.
        '''
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose

        self.path.header = odom.header
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.drive_stop()
        rospy.loginfo("Stopping")
        rospy.sleep(1)

def main():
    rospy.init_node('follower', anonymous=True)
    Follower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
