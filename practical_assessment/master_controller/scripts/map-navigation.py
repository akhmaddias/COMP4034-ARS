import rospy
import matplotlib.pyplot as plt

from matplotlib import colors, animation
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point

WAIT_TIME = 5

class Navigation():
    '''
    Moves the bot acrosss the set waypoint
    '''

    def drive(self, waypoint):
        '''
        Move to set coordinates
        '''

        rospy.loginfo("Moving to waypoint %s.", waypoint.room_number)
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(waypoint[0], waypoint[1], 0)
        goal.target_pose.pose.orientation = waypoint[2]
        
        self.action.send_goal(goal)
        while not rospy.is_shutdown():
            self.action.wait_for_result(rospy.Duration(WAIT_TIME))
            state = self.action.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True
            elif state == GoalStatus.ACTIVE:
                rospy.loginfo("Moving")
            else:
                return False

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

    