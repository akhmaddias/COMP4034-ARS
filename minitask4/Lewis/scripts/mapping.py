#!/usr/bin/env python
'''
Will create a discretised environment to keep track of
where the Turtlebot3 Waffle has visited.
'''

import random

import rospy

import numpy as np
import matplotlib.pyplot as plt

from math import floor
from matplotlib import colors, animation
from nav_msgs.msg import Odometry

# Grid parameters
ORIGIN_X = -10.0
ORIGIN_Y = -10.0
SIZE_X = 100
SIZE_Y = 100
RESOLUTION = 0.2
VISITED = 0
UNVISITED = -1

class OccupancyGrid():
    '''
    Will create a discretised environment to keep track of
    where the Turtlebot3 Waffle has visited.
    '''

    def __init__(self):
        rospy.init_node('occupancy_grid', anonymous=False)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.initialise_grid(UNVISITED)
        self.setup_plot()

    def odom_callback(self, odom):
        '''
        Callback method for the odometry subscriber.
        '''
        world_x, world_y = odom.pose.pose.position.x, odom.pose.pose.position.y

        (grid_x, grid_y) = to_grid(world_x, world_y)
        cell_index = to_cell_index(grid_x, grid_y)

        self.grid_cells[cell_index] = VISITED

    def initialise_grid(self, unvisited_character):
        '''
        Creates an array of the resolution of the grid
        and the fills it with the UNVISITED character
        '''
        self.grid_cells = np.full((SIZE_X * SIZE_Y), unvisited_character)

    def setup_plot(self):
        '''
        Displays a graph representing the occupancy grid
        and draws it using the animation.
        '''
        figure, plot = plt.subplots()

        cmap = colors.ListedColormap(['gray', 'white', 'black'])
        bounds = [-1, 0, 1, 2]
        self.image = plot.imshow(
            self.grid_cells.reshape(SIZE_X, SIZE_Y),
            cmap = cmap,
            norm = colors.BoundaryNorm(bounds, cmap.N))

        # Clears axis labels
        plt.tick_params(
            axis = 'both',
            which = 'both',
            bottom = False,
            left = False,
            labelbottom = False,
            labelleft = False)

        figure.set_size_inches((8.5, 11), forward = False)  # Sets figure size

        _ = animation.FuncAnimation(figure, self.animate, blit = True)  # Draws graph

        plt.show()

    def animate(self, _):
        '''
        Redraws the plot using the current data and returns
        it in a weird and wonderful way that pyplot understands.
        '''
        self.image.set_data(self.grid_cells.reshape(SIZE_X, SIZE_Y))
        return self.image,

def to_grid(world_x, world_y):
    '''
    Will return occupancy grid coordinates given real-world coordinates
    or None if the coordinates fall outside the range of the occupancy grid.
    '''
    if (world_x < ORIGIN_X or world_y < ORIGIN_Y):
        return None  # If the coordinates are before the start of the grid

    grid_x = floor((world_x - ORIGIN_X) / RESOLUTION)
    grid_y = floor((world_y - ORIGIN_Y) / RESOLUTION)

    if (grid_x >= SIZE_X or grid_y >= SIZE_Y):
        return None  # If the coordinates are after the end of the grid

    return (grid_x, grid_y)

def to_world(grid_x, grid_y):
    '''
    Will return real-world coordinates given occupancy grid coordinates
    or None if the coordinates fall outside the occupancy grid.
    '''
    if (grid_x > SIZE_X or grid_y > SIZE_Y):
        return None  # If the coordinates are after the end of the grid

    world_x = (grid_x + ORIGIN_X) * RESOLUTION
    world_y = (grid_y + ORIGIN_Y) * RESOLUTION

    if (world_x < ORIGIN_X or world_y < ORIGIN_Y):
        return None  # If the coordinates are before the start of the grid

    return (world_x, world_y)

def to_cell_index(grid_x, grid_y):
    '''
    Will return an array index given a grid coordinates.
    This is mirrored in X and Y so that the map accurately represents
    the path that the bot has followed.
    '''
    return int((SIZE_X * SIZE_Y) - (grid_x * SIZE_X + grid_y))

if __name__ == '__main__':
    try:
        OccupancyGrid()
    except rospy.ROSInterruptException:
        pass
