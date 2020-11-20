#!/usr/bin/env python
import rospy
import random
import numpy as np
import matplotlib.pyplot as plt

from math import floor
from matplotlib import colors, animation
from nav_msgs.msg import Odometry

# define grid
ORIGIN_X = -10.0
ORIGIN_Y = -10.0
SIZE_X = 20
SIZE_Y = 20
RESOLUTION = 1

CMAP = colors.ListedColormap(['gray', 'white', 'black'])
BOUNDS = [-1, 0, 1, 2]
NORM = colors.BoundaryNorm(BOUNDS, CMAP.N)


class OccupancyGrid():

    def __init__(self):
        rospy.init_node('occupancy_grid', anonymous=False)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        # array of grid cells (initially filled with -1)
        self.cells = np.full((SIZE_X * SIZE_Y), -1)

        fig, ax = plt.subplots()
        self.im = ax.imshow(self.cells.reshape(SIZE_X, SIZE_Y),
                            cmap=CMAP, norm=NORM)
        ax.grid(which='major', axis='both', linestyle='-', color='k',
                linewidth=1)
        ax.set_xticks(np.arange(0.5, SIZE_X, 1))
        ax.set_yticks(np.arange(0.5, SIZE_Y, 1))
        plt.tick_params(axis='both', which='both', bottom=False,
                        left=False, labelbottom=False, labelleft=False)
        fig.set_size_inches((8.5, 11), forward=False)
        # animation
        anim = animation.FuncAnimation(fig, self.animate, blit=True)
        plt.show()

    def to_index(self, gx, gy):
        return int(gy * SIZE_X + gx)

    def to_grid(self, px, py):
        if (px < ORIGIN_X or py < ORIGIN_Y):
            return None
        gx = floor((px - ORIGIN_X) / RESOLUTION)
        gy = floor((py - ORIGIN_Y) / RESOLUTION)

        if (gx >= SIZE_X or gy >= SIZE_Y):
            return None

        return (gx, gy)

    def to_world(self, gx, gy):
        if (gx > SIZE_X or gy > SIZE_Y):
            return None
        px = (gx + ORIGIN_X) * RESOLUTION
        py = (gy + ORIGIN_Y) * RESOLUTION
        if (px < ORIGIN_X or py < ORIGIN_Y):
            return None
        return (px, py)

    def animate(self, _):
        self.im.set_data(self.cells.reshape(SIZE_X, SIZE_Y))
        return self.im,

    def odom_cb(self, odom):
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        (gx, gy) = self.to_grid(x, y)
        cell_index = self.to_index(gx, gy)
        self.cells[cell_index] = 0


if __name__ == '__main__':
    try:
        OccupancyGrid()
    except rospy.ROSInterruptException:
        pass
