#!/usr/bin/env python

__author__ = 'Ben Henaghan'

import rospy 

from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


SAMPLE_WINDOW = 90
COLLISION_DISTANCE = 0.25
SPEED = 0.1
TURN_SPEED = 0.1

MESSAGE_BLOCKED = 1
MESSAGE_UNBLOCKED = 0

class CollisionAvoidance():

    def __init__(self):
        rospy.init_node('collision_avoidance', anonymous=False)

        self.message_pub = rospy.Publisher("collision_control", Int32, queue_size=1)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        self.blocked = False
        self.backward_twist = Twist()
        self.backward_twist.linear.x = -SPEED

        self.spin_twist = Twist()
        self.spin_twist.angular.z = TURN_SPEED

    def sample_scan(self, scandata, target, window_size=SAMPLE_WINDOW):
        '''
        Returns the minimum value in the given window_size, centered on target
        '''
        assert type(window_size) == int, "window_size should be an integer"
        offset = window_size / 2
        # Was going to use min() and list slicing but its a pain with wrapping around
        min_val = scandata.ranges[target - offset]
        for i in range(target - offset + 1, target + offset):
            min_val = min(scandata.ranges[i], min_val)
        return min_val

    def scan_cb(self, scandata):
        closest_val = self.sample_scan(scandata, 0)

        if closest_val < COLLISION_DISTANCE:

            if not self.blocked:
                rospy.loginfo('Blocked!')
                self.blocked = True
                self.message_pub.publish(MESSAGE_BLOCKED)

            back_dist = self.sample_scan(scandata, 180)
            if back_dist > COLLISION_DISTANCE:
                rospy.loginfo('Driving backward')
                self.vel_publisher.publish(self.backward_twist)
            else:
                rospy.loginfo('Backwars is blocked, spinning on the spot!')
                self.vel_publisher.publish(self.spin_twist)
        elif self.blocked:
            rospy.loginfo("Collision Avoided!")
            self.blocked = False
            self.vel_publisher.publish(Twist())
            self.message_pub.publish(MESSAGE_UNBLOCKED)


if __name__ == '__main__':
    CollisionAvoidance()
    rospy.spin()
            
                
            
        