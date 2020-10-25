#!/usr/bin/env python
import rospy
import tf

from math import pi as PI, sqrt

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose


def get_forward_twist():
    t = Twist()
    t.linear.x = 0.25
    t.linear.y = 0
    t.linear.z = 0

    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0

    return t


def get_turning_twist():
    t = Twist()
    t.linear.x = 0
    t.linear.y = 0
    t.linear.z = 0

    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = ((2 * PI) / 360) * 45

    return t


def get_stop_twist():
    t = Twist()
    t.linear.x = 0
    t.linear.y = 0
    t.linear.z = 0

    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0

    return t


def drive_open_loop():
    '''
    Drive the turtlebot in a 1m square using open-loop control.
    '''
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.init_node('mover_square_open', anonymous=True)

    start_plot()

    r = rospy.Rate(0.5)
    start_t = rospy.Time.now()

    straight = get_forward_twist()
    turn = get_turning_twist()
    stop = get_stop_twist()

    phase = 0
    while rospy.Time.now() - start_t < rospy.Duration(32000) \
            and phase <= 16 \
            and not rospy.is_shutdown():
        if phase % 4 == 0:
            print("forward")
            pub.publish(straight)
        elif phase % 2 == 1:
            print("stop")
            pub.publish(stop)
        else:
            print("turn")
            pub.publish(turn)
        phase += 1
        r.sleep()

    pub.publish(stop)


pub = None
last_pose = None
turning = False
progress = 0
turns = 0


def odom_callback(odom):
    global last_pose
    global turning
    global turns
    global progress
    # Get (x, y, theta) specification from odometry topic
    quarternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                   odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

    p = {}
    p["theta"] = yaw
    p["x"] = odom.pose.pose.position.x
    p["y"] = odom.pose.pose.position.y
    # print(progress, turning, p["theta"])
    
    if turns >= 4:
        pub.publish(get_stop_twist())
        rospy.signal_shutdown("Done!")
        return


    if last_pose:
        if turning:
            progress += abs(p["theta"] - last_pose["theta"])
            if progress > PI / 2:
                turning = False
                pub.publish(get_stop_twist())
                progress = 0
                turns += 1
            else:
                pub.publish(get_turning_twist())
        else:
            progress += sqrt((last_pose["x"] - p["x"]) ** 2 +
                             (last_pose["y"] - p["y"]) ** 2)
            if progress > 1:
                turning = True
                pub.publish(get_stop_twist())
                progress = 0
            else:
                pub.publish(get_forward_twist())
    last_pose = p


def drive_closed_loop():
    global pub
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.init_node('mover_square_open', anonymous=True)

    start_plot()

    odom_subscriber = rospy.Subscriber("/odom", Odometry, odom_callback)
    pub.publish(get_forward_twist())
    rospy.spin()


path_publisher = None
path = None


def start_plot():
    global path_publisher
    global path

    path = Path()

    odom_subscriber = rospy.Subscriber("/odom", Odometry, plot_trajectory)
    path_publisher = rospy.Publisher("/path", Path, queue_size=10)


def plot_trajectory(odom):
    global path

    p = PoseStamped()
    p.header = odom.header
    p.pose = odom.pose.pose

    path.header = odom.header
    path.poses.append(p)

    path_publisher.publish(path)


if __name__ == '__main__':
    try:
        drive_closed_loop()
    except rospy.ROSInterruptException:
        pass
