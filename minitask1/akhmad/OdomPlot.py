#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class OdomPlot():
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        self.xs.append(x)
        self.ys.append(y)

    def plot_trajectory(self, f):
        self.line.set_data(self.xs, self.ys)
        return self.line,

    def __init__(self):
        # init
        rospy.init_node('odom_plot', anonymous=True)

        # init interactive plot
        self.xs = []
        self.ys = []
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.line, = self.ax.plot([], [])

        # shutdown (fn to exec when ctrl + c)
        rospy.on_shutdown(self.shutdown)

        # subscriber
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # animation
        ani = animation.FuncAnimation(self.fig, self.plot_trajectory, blit=True)
        plt.show()

    def shutdown(self):
        rospy.sleep(1)
        

if __name__ == '__main__':
    try:
        OdomPlot()
    except rospy.ROSInterruptException:
        pass