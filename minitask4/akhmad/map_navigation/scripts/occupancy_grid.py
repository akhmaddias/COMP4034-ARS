#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

from math import floor
from matplotlib import colors, animation
from nav_msgs.msg import Odometry

# define grid
ORIGIN_X = -10.0
ORIGIN_Y = -10.0
SIZE_X = 100
SIZE_Y = 100
RESOLUTION = 0.2
VISITED = 0
UNVISITED = -1

CMAP = colors.ListedColormap(['gray', 'white', 'black'])
BOUNDS = [-1, 0, 1, 2]
NORM = colors.BoundaryNorm(BOUNDS, CMAP.N)


class OccupancyGrid():

    def __init__(self):
        rospy.init_node('occupancy_grid', anonymous=False)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # array of grid cells (initially filled with -1)
        self.cells = np.full((SIZE_X * SIZE_Y), UNVISITED)

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
        _ = animation.FuncAnimation(fig, self.animate, blit=True)
        plt.show()

    def to_index(self, gx, gy):
        return int((SIZE_X * SIZE_Y) - (gx * SIZE_X + gy))

    def to_grid(self, px, py):
        if (px < ORIGIN_X or py < ORIGIN_Y):
            return None
        gx = floor((px - ORIGIN_X) / RESOLUTION)
        gy = floor((py - ORIGIN_Y) / RESOLUTION)

        if (gx >= SIZE_X or gy >= SIZE_Y):
            return None

        return (gx, gy)

    def animate(self, _):
        self.im.set_data(self.cells.reshape(SIZE_X, SIZE_Y))
        return self.im,

    def odom_cb(self, odom):
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        (gx, gy) = self.to_grid(x, y)
        cell_index = self.to_index(gx, gy)
        self.cells[cell_index] = VISITED


if __name__ == '__main__':
    try:
        OccupancyGrid()
    except rospy.ROSInterruptException:
        pass
