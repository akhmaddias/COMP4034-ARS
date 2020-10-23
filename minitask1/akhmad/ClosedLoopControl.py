#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tf.transformations import euler_from_quaternion


class ClosedLoopControl():
    def message(self, constraint, msg):
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < rospy.Duration(time).to_sec():
            self.cmd_vel_pub.publish(msg)
            rate.sleep()

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quarternion)

        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def __init__(self):
        # init
        self.current_theta = 0
        self.theta = 0
        self.x = 0
        self.y = 0
        rospy.init_node('move_square', anonymous=True)

        # shutdown (fn to exec when ctrl + c)
        rospy.on_shutdown(self.shutdown)

        # publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # subscriber
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # rate
        rate = rospy.Rate(10)

        # message to move forward with speed 0.1m/s
        move_forward_msg = Twist()
        move_forward_msg.linear.x = 0.1

        # message to turn 15deg/s
        turn_msg = Twist()
        turn_msg.linear.x = 0
        turn_msg.angular.z = math.radians(15)

        
        while self.x < 1:
            self.cmd_vel_pub.publish(move_forward_msg)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

        while abs(self.current_theta - self.theta) < math.pi / 2:
            self.cmd_vel_pub.publish(turn_msg)
            rate.sleep()
        self.current_theta = self.theta
        self.cmd_vel_pub.publish(Twist())

        while self.y < 1:
            self.cmd_vel_pub.publish(move_forward_msg)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())
        
        while abs(self.current_theta - self.theta) < math.pi / 2:
            self.cmd_vel_pub.publish(turn_msg)
            rate.sleep()
        self.current_theta = self.theta
        self.cmd_vel_pub.publish(Twist())

        while self.x > 0:
            self.cmd_vel_pub.publish(move_forward_msg)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())
        
        while abs(self.current_theta - self.theta) < math.pi / 2:
            self.cmd_vel_pub.publish(turn_msg)
            rate.sleep()
        self.current_theta = self.theta
        self.cmd_vel_pub.publish(Twist())

        while self.y > 0:
            self.cmd_vel_pub.publish(move_forward_msg)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        

if __name__ == '__main__':
    try:
        ClosedLoopControl()
    except rospy.ROSInterruptException:
        pass