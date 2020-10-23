#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import radians


class TurtleBotDriving():
    def message(self, time, rate, msg):
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < rospy.Duration(time).to_sec():
            self.cmd_vel_pub.publish(msg)
            rate.sleep()

    def __init__(self):
        # init
        rospy.init_node('move_square', anonymous=True)

        # shutdown (fn to exec when ctrl + c)
        rospy.on_shutdown(self.shutdown)

        # publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # rate
        rate = rospy.Rate(10)

        # message to move forward with speed 0.2m/s
        move_forward_msg = Twist()
        move_forward_msg.linear.x = 0.2

        # message to turn 45deg/s
        turn_msg = Twist()
        turn_msg.linear.x = 0
        turn_msg.angular.z = radians(45)

        while not rospy.is_shutdown():
            # move forward 5s * 0.2m/s = 1m
            self.message(5, rate, move_forward_msg)

            # turn 2s * 45deg/s = 90deg
            self.message(2, rate, turn_msg)

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        

if __name__ == '__main__':
    try:
        TurtleBotDriving()
    except rospy.ROSInterruptException:
        pass