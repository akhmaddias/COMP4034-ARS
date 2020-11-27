#!/usr/bin/env python
'''
Will automatically localise a Turtlebot3 Waffle in minitask4
'''

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def initial_pos_pub():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('initial_pos_pub', anonymous=True)
    #Creating the message with the type PoseWithCovarianceStamped
    start_pos = PoseWithCovarianceStamped()
    start_pos.header.frame_id = "map"

    start_pos.pose.pose.position.x = -3.0
    start_pos.pose.pose.position.y = 1.0
    start_pos.pose.pose.position.z = 0.0

    rospy.loginfo(start_pos)
    publisher.publish(start_pos)

if __name__ == '__main__':
    try:
        initial_pos_pub()
    except rospy.ROSInterruptException:
        pass
