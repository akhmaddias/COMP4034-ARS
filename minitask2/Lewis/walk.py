#!/usr/bin/env python
import rospy
import tf

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from math import radians, pi as PI, sqrt, cos, sin, atan
from random import uniform

FORWARD_SPEED = 0.2
TURN_SPEED = ((2 * PI) / 360) * 25  # 25 Deg/s
WALK_DISTANCE = 3.0
OFFSET_ERROR_LINEAR = 0.01  # Allowable offset from target is 0.01 meters
OFFSET_ERROR_ANGULAR = ((2 * PI) / 360) * 0.1   # Allowable offset from target is 0.1 degrees

class Walk():

    def __init__(self):
        rospy.init_node('Walk', anonymous=True)
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
        self.turn_left.linear.x = 0
        self.turn_left.angular.z = TURN_SPEED
        self.turn_right = Twist()
        self.turn_left.linear.x = 0
        self.turn_right.angular.z = -TURN_SPEED
        self.travel_angle = 0.0
        
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

        # For behavioural blocking
        self.blocked = False

    def drive_forward(self):
        self.vel_publisher.publish(self.forward)

    def drive_turn_left(self):
        self.vel_publisher.publish(self.turn_left)

    def drive_turn_right(self):
        self.vel_publisher.publish(self.turn_right)

    def drive_stop(self):
        self.vel_publisher.publish(Twist())

    def random_walk(self, distance = 3):
        '''
        Waffle moves forward 3 meters, randomly chooses a direction and rotates to continue the random walk. 
        This behaviour is overriden by wall and obstacle detection.
        '''
        rospy.loginfo("Random Walk")
        
        while not self.blocked:
            if self.turning:
                if (self.travel_angle < 0 and self.curr_pose["theta"] > self.travel_angle) or (self.travel_angle >= 0 and self.curr_pose["theta"] < self.travel_angle):
                    if self.travel_angle > 0:
                        self.drive_turn_left()
                    else:
                        self.drive_turn_right()
                else:
                    rospy.loginfo("Driving")
                    self.turning = False
                    self.drive_stop()
            else:

                progress = sqrt((self.last_pose["x"] - self.curr_pose["x"])** 2 + (self.last_pose["y"] - self.curr_pose["y"])** 2)
                if progress <= distance:
                    self.drive_forward()
                    self.turning = False
                else:
                    rospy.loginfo("Turning")
                    self.drive_stop()
                    self.travel_angle = uniform(-PI, PI)
                    self.set_location_ref()
                    self.set_angle()
                    self.turning = True

    def set_angle(self):
        self.travel_angle = uniform(-PI, PI)
        rospy.loginfo("Travel angle: %f", self.travel_angle)

    def set_location_ref(self):
        rospy.loginfo("Recording last position")
        self.last_pose["theta"] = self.curr_pose["theta"]
        self.last_pose["x"] = self.curr_pose["x"]
        self.last_pose["y"] = self.curr_pose["y"]
        rospy.loginfo("Theta: %f, x: %f, y: %f", self.last_pose["theta"], self.last_pose["x"], self.last_pose["y"])

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

    def start(self):
        rospy.loginfo("Waiting for odometry")
        while self.curr_pose["x"] == 0 or self.curr_pose["y"] == 0:
            self.drive_forward()
        rospy.loginfo("Receiving odometry")
        self.drive_stop()
        self.set_location_ref()
        self.random_walk()
        
        rospy.spin()

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.blocked = True
        self.vel_publisher.publish(Twist())
        rospy.loginfo("Stopping")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Walk().start()
    except rospy.ROSInterruptException:
        pass
