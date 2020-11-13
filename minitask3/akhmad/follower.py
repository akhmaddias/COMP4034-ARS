#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan

LOWER_GREEN = numpy.array([60, 100, 50])
UPPER_GREEN = numpy.array([60, 255, 255])

SAMPLE_WINDOW = 20  # Degrees

TURN_MAX_SPEED = 0.2
TURN_GAIN = 0.01

FORWARD_SPEED = 0.3


class Follower():

    def __init__(self):
        rospy.init_node('robot_control', anonymous=True)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bridge = cv_bridge.CvBridge()

        self.move_twist = Twist()

        self.front = False
        self.right = False
        self.green_block_detected = False
        self.centroids = []
        self.width_middle = None

    def start(self):
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.onscan)
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                           Image, self.onimage)
        rospy.spin()

    def move(self, turnspeed, forwardspeed):
        '''
        Move the robot forward at forwardspeed, and turn at turnspeed.
        '''
        self.move_twist.linear.x = forwardspeed
        self.move_twist.angular.z = turnspeed
        self.vel_pub.publish(self.move_twist)

    def sample_scan(self, scandata, target, window_size=SAMPLE_WINDOW):
        '''
        Returns the minimum value in the given window_size, centered on target
        '''
        assert type(window_size) == int, "window_size should be an integer"
        offset = window_size / 2

        min_val = scandata.ranges[target - offset]
        for i in range(target - offset + 1, target + offset):
            min_val = min(scandata.ranges[i], min_val)
        return min_val

    def move_by_image(self):
        rospy.loginfo("Walking towards green block")
        err_x = float('inf')
        err_y = float('inf')

        # Walk towards block that is closest to middle of image width
        for c in self.centroids:
            err_x = min(c['cx'] - self.width_middle, err_x)
            err_y = min(c['cy'] - self.height_middle, err_y)
        turnspeed = -float(err_x) * TURN_GAIN

        # Make sure turnspeed is not too high
        turnspeed = TURN_MAX_SPEED if turnspeed > TURN_MAX_SPEED else turnspeed
        turnspeed = -TURN_MAX_SPEED if turnspeed < -TURN_MAX_SPEED else turnspeed

        # Was trying to stop robot in front of block but this doesn't work
        if (err_y > -10 and err_y < 10 and self.front):
            self.move(0, 0)
        else:
            self.move(turnspeed, FORWARD_SPEED)

    def move_by_scan(self):
        rospy.loginfo("Just walking")
        if self.front:
            self.move(TURN_MAX_SPEED if self.right else -TURN_MAX_SPEED, 0)
        else:
            self.move(0, FORWARD_SPEED)

    def onimage(self, imagedata):
        image = self.bridge.imgmsg_to_cv2(imagedata, desired_encoding='bgr8')
        image_resized = cv2.resize(image, (image.shape[1]/4, image.shape[0]/4))

        self.width_middle = image_resized.shape[1]/2
        self.height_middle = image_resized.shape[0]/2
        self.centroids = []

        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                          cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.centroids.append({'cx': cx, 'cy': cy})
                self.green_block_detected = True
                cv2.circle(image_resized, (cx, cy), 5, (0, 0, 255), -1)

        cv2.imshow("binary", image_resized)
        cv2.waitKey(3)
        if self.green_block_detected:
            self.move_by_image()

    def onscan(self, scandata):
        self.front = self.sample_scan(scandata, 0, window_size=60) <= 0.5
        self.right = self.sample_scan(scandata, 270, window_size=60) <= 0.5
        if self.front or not self.green_block_detected:
            self.move_by_scan()


if __name__ == "__main__":
    try:
        Follower().start()
    except rospy.ROSInterruptException:
        pass
