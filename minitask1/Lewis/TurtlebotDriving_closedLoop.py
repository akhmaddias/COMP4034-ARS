#!/usr/bin/env python
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
from tf.transformations import euler_from_quaternion

# Drives the Turtlebot in a 1m sqaure using open-loop control.
class TurtlebotDriving_closedLoop():
    
    def __init__(self):
        self.ongoingTheta = 0
        self.currentTheta = 0
        self.x = 0
        self.y = 0
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.odomSubscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.init_node('TurtlebotDriving_closedLoop', anonymous = True)
        
        # Forward message to move the Turtlebot forward at 0.1m/s
        forwardMessage = Twist()
        forwardMessage.linear.x = 0.1

        # Left message to turn the Turtlebot by 10 degrees.
        leftMessage = Twist()
        leftMessage.angular.z = radians(10)

        # Stop message to stop the Turtlebot
        stopMessage = Twist()
        stopMessage.linear.x = 0
        stopMessage.angular.z = 0

        # Variables to keep track of square drawing progress
        sideCount = 0
        turnCount = 0

        # Driver
        # Forward by 1m
        while self.x < 1:
            self.pub.publish(forwardMessage)
        
        self.pub.publish(stopMessage)
        sideCount += 1

        # Left by 90 degrees
        while abs(self.currentTheta - self.ongoingTheta) < radians(90):
            self.pub.publish(leftMessage) 
        
        self.pub.publish(stopMessage)
        self.currentTheta = self.ongoingTheta
        turnCount += 1

        # Forward by 1m
        while self.y < 1:
            self.pub.publish(forwardMessage)
        
        self.pub.publish(stopMessage)
        sideCount += 1

        # Left by 90 degrees
        while abs(self.currentTheta - self.ongoingTheta) < radians(90):
            self.pub.publish(leftMessage) 
        
        self.pub.publish(stopMessage)
        self.currentTheta = self.ongoingTheta
        turnCount += 1

        # Forward by 1m
        while self.x > 0:
            self.pub.publish(forwardMessage)
        
        self.pub.publish(stopMessage)
        sideCount += 1

        # Left by 90 degrees
        while abs(self.currentTheta - self.ongoingTheta) < radians(90):
            self.pub.publish(leftMessage) 
        
        self.pub.publish(stopMessage)
        self.currentTheta = self.ongoingTheta
        turnCount += 1

         # Forward by 1m
        while self.y > 0:
            self.pub.publish(forwardMessage)
        
        self.pub.publish(stopMessage)
        sideCount += 1

        # Left by 90 degrees
        while abs(self.currentTheta - self.ongoingTheta) < radians(90):
            self.pub.publish(leftMessage) 
        
        self.pub.publish(stopMessage)
        self.currentTheta = self.ongoingTheta
        turnCount += 1


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.ongoingTheta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def message(self, timeToSend, data):
        rate = rospy.Rate(10)
        currentTime = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - currentTime) < rospy.Duration(timeToSend).to_sec():
            self.pub.publish(data)
            rate.sleep()

if __name__ == '__main__':
    try:
        TurtlebotDriving_closedLoop()
    except rospy.ROSInterruptException:
        pass