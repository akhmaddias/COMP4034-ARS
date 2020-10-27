#!/usr/bin/env python
import rospy
import tf

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose

from math import pi as PI, sqrt

NINETY_DEGREES = PI / 2
FORWARD_SPEED = 0.125
TURN_SPEED = ((2 * PI) / 360) * 25  # 25 Deg/s


class ClosedLoopController():

    def __init__(self):
        self.vel_publisher = None
        self.last_pose = None
        self.turning = False
        self.progress = 0
        self.turns = 0
        self.square_size = 0

        self.stop = Twist()

        self.forward = Twist()
        self.forward.linear.x = FORWARD_SPEED

        self.turn = Twist()
        self.turn.angular.z = TURN_SPEED

    def drive_forward(self):
        self.vel_publisher.publish(self.forward)

    def drive_turn(self):
        self.vel_publisher.publish(self.turn)

    def drive_stop(self):
        self.vel_publisher.publish(self.stop)

    def odom_callback(self, odom):
        # Get (x, y, theta) specification from odometry topic
        quart = [odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quart)

        curr_pose = {}
        curr_pose["theta"] = yaw
        curr_pose["x"] = odom.pose.pose.position.x
        curr_pose["y"] = odom.pose.pose.position.y

        if self.turns >= 4:
            self.drive_stop()
            rospy.signal_shutdown("Done!")
            return

        if self.last_pose:
            if self.turning:
                self.progress += abs(curr_pose["theta"] - self.last_pose["theta"])
                if self.progress > NINETY_DEGREES:
                    self.turning = False
                    self.drive_stop()
                    self.progress = 0
                    self.turns += 1
                else:
                    self.drive_turn()
            else:
                self.progress += sqrt((self.last_pose["x"] - curr_pose["x"]) ** 2 +
                                      (self.last_pose["y"] - curr_pose["y"]) ** 2)
                if self.progress > self.square_size:
                    self.turning = True
                    self.drive_stop()
                    self.progress = 0
                else:
                    self.drive_forward()

        self.last_pose = curr_pose

    def drive(self, square_size=1):
        self.square_size = square_size

        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.init_node('mover_square_open', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.start_plot()
        self.drive_forward()
        rospy.spin()

    def start_plot(self):
        pass  # TODO integrate the final plotting


if __name__ == '__main__':
    try:
        ClosedLoopController().drive()
    except rospy.ROSInterruptException:
        pass
