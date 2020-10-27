#!/usr/bin/env python
import rospy
import tf

from geometry_msgs.msg import Twist
from math import radians

# Drives the Turtlebot in a 1m sqaure using open-loop control.
class TurtlebotDriving():
    
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        rospy.init_node('TurtlebotDriving', anonymous = True)

        rospy.on_shutdown(self.shutdown)

        self.drive()

    
    # Sends the appropriate commands to the robot via the msg
    def drive(self):
        # Forward message to move the Turtlebot forward at 0.1m/s
        forwardMessage = Twist()
        forwardMessage.linear.x = 0.1
        forwardMessage.angular.z = 0

        # Left message to turn the Turtlebot by 10 degrees.
        leftMessage = Twist()
        leftMessage.linear.x = 0
        leftMessage.angular.z = radians(10)

        # Stop message to stop the Turtlebot
        stopMessage = Twist()
        stopMessage.linear.x = 0
        stopMessage.angular.z = 0

        # Variables to keep track of square drawing progress
        sideCount = 0
        turnCount = 0

        while not rospy.is_shutdown() and sideCount < 4 and turnCount < 4:
            # Move forwards by 1m
            print("[TWIST]  Forward")
            self.msg(10, forwardMessage)
            sideCount += 1
            print("[LOG]    Sides completed: " + str(sideCount))

            # Turn by 90 degrees
            print ("[TWIST]  Left")
            self.msg(9, leftMessage)
            turnCount += 1
            print("[LOG]    Turns completed: " + str(turnCount))

        print("[TWIST]  Stop")
        self.msg(1, stopMessage)


    # Messenger to send data to the robot
    def msg(self, timeToSend, data):
        rate = rospy.Rate(10)
        currentTime = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - currentTime) < rospy.Duration(timeToSend).to_sec():
            self.pub.publish(data)
            rate.sleep()


    # Shutdown method in the event of ctrl + c
    def shutdown(self):
        self.pub.publish(Twist())
        print("[EXIT]  Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        TurtlebotDriving()
    except rospy.ROSInterruptException:
        pass