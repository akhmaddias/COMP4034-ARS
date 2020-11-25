#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt

from matplotlib import colors, animation
from nav_msgs.msg import Odometry

GRID_UNVISITED = -1
GRID_VISITIED = 0


class OccupancyGrid():

    SIZE_X = 400
    SIZE_Y = 400

    def __init__(self, origin=(0, 0), resolution=1):
        rospy.init_node('occupancy_grid', anonymous=True)
        self.odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.odom_cb)

        self.origin = origin
        self.resolution = resolution
        self.grid = np.full((OccupancyGrid.SIZE_X * OccupancyGrid.SIZE_Y),
                            GRID_UNVISITED)

# STOLEN FROM LEWIS: Couldn't figure out a better/interesting way to do this so I've just stolen Lewis's code
    def setup_plot(self):
        '''
        Displays a graph representing the occupancy grid
        and draws it using the animation.
        '''
        figure, plot = plt.subplots()

        cmap = colors.ListedColormap(['gray', 'white', 'black'])
        bounds = [-1, 0, 1, 2]
        self.image = plot.imshow(
            self.grid.reshape(SIZE_X, SIZE_Y),
            cmap=cmap,
            norm=colors.BoundaryNorm(bounds, cmap.N))

        # Clears axis labels
        plt.tick_params(
            axis='both',
            which='both',
            bottom=False,
            left=False,
            labelbottom=False,
            labelleft=False)

        figure.set_size_inches((8.5, 11), forward=False)  # Sets figure size
        animation.FuncAnimation(figure, self.animate, blit=True)  # Draws graph
        plt.show()

    def animate(self, _):
        '''
        Redraws the plot using the current data and returns
        it in a weird and wonderful way that pyplot understands.
        '''
        self.image.set_data(self.grid.reshape(SIZE_X, SIZE_Y))
        return self.image
# END OF STOLEN FROM LEWIS


    def odom_cb(self, odom):
        x, y, _ = odom.pose.pose.position
        grid_pos = self.to_grid(x, y)
        index = self.to_cell_index(*grid_pos)

    def to_grid(self, x, y):
        return self.to_grid((x, y))

    def to_grid(self, pos):  # Code golf incoming
        return tuple((pos[dim] - self.origin[dim]) / self.resolution for dim in (0, 1))

    def to_world(self, x, y):
        return self.to_world((x, y))

    def to_world(self, pos):
        return tuple((pos[dim] + self.origin[dim]) * self.resolution for dim in (0, 1))

    def to_cell_index(self, x, y):
        return (OccupancyGrid.SIZE_X * OccupancyGrid.SIZE_Y) - \
            (x * OccupancyGrid.SIZE_X) - y
