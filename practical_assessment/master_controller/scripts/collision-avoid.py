import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from abc import abstractmethod
from math import pi as PI, radians, degrees, sqrt, cos

THREE_METERS = 3

SAMPLE_WINDOW = 20  # Degrees

TURNING_ERROR_THRESHOLD = radians(10)
TURN_GAIN = 0.5
TURN_MAX_SPEED = 1
TURN_MIN_SPEED = 0.01

FORWARD_GAIN = 0.4
FORWARD_MAX_SPEED = 0.3
FORWARD_MIN_SPEED = 0.1

class RobotBehaviour(object):
    '''
    Abstract class that other robot behaviours inherit from.
    '''

    def __init__(self, vel_pub):
        self.vel_pub = vel_pub
        self.move_twist = Twist()
        self.name = "!!!"

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

    def move(self, turnspeed, forwardspeed):
        '''
        Move the robot forward at forwardspeed, and turn at turnspeed.
        '''
        self.move_twist.linear.x = forwardspeed
        self.move_twist.angular.z = turnspeed
        self.vel_pub.publish(self.move_twist)

    def turn(self, speed):
        '''
        Turn the robot at turnspeed, a positive number is leftwards (CCW).
        '''
        self.move(speed, 0)

    def forward(self, speed):
        '''
        Move the robot forwards at the given speed.
        '''
        self.move(0, speed)

    def stop(self):
        '''
        Stop the robot.
        '''
        self.move(0, 0)

    @abstractmethod
    def shouldrun(self, scandata):
        '''
        Returns a boolean stating whether this behaviour wants to run or not.
        '''
        raise NotImplementedError

    @abstractmethod
    def run(self, scandata):
        '''
        A method to publish instrunctions for each behaviour.
        '''
        raise NotImplementedError

    @abstractmethod
    def clear_behaviour(self):
        '''
        Clears any relevant instance variables of the behaviour (eg targets,
        progress etc).
        '''
        raise NotImplementedError


class ObstacleAvoidance(RobotBehaviour):
    '''
    This behvaiour will turn right if it encounters an object closer than 0.5m
    '''

    TURN_SPEED = 0.3

    def __init__(self, vel_pub):
        super(ObstacleAvoidance, self).__init__(vel_pub)
        self.name = "Obstacle Avoidance"

    def shouldrun(self, scandata):
        '''
        Returns true if there is an object closer than 0.5m in front of the robot.
        '''
        return super(ObstacleAvoidance, self).sample_scan(scandata,
                                                          0,
                                                          window_size=60) <= 0.5

    def run(self, scandata):
        '''
        This behaviour will always turn if it needs to run. It will only return
        True from shouldrun if there is a wall in front, so we only ever need
        to turn.
        '''
        super(ObstacleAvoidance, self).turn(ObstacleAvoidance.TURN_SPEED)

    def clear_behaviour(self):
        pass  # Not needed


class WallFollowing(RobotBehaviour):
    '''
    This behaviour will follow any wall on the right-hand side of the robot

    The wall must be within START_THRESHOLD for the wall following to start.
    It will stop if the wall is END_THRESHOLD away. It will also try to keep
    about IDEAL_DIST away from the wall.

    It uses a closed-loop proportional control for the turn speed to achieve
    this.
    '''

    START_THRESHOLD = 1.0
    IDEAL_DIST = 1.0
    END_THRESHOLD = 1.5
    SAMPLE_ANGLE = 10
    WINDOW = 8  # Degrees

    MOVE_SPEED = 0.1
    TURN_GAIN = 14

    def __init__(self, vel_pub):
        super(WallFollowing, self).__init__(vel_pub)  # Python2 is bad
        self.name = "Wall Following"
        self.started = False

    def shouldrun(self, scandata):
        '''
        Takes 3 samples, if they are all under the start threshold, then start
        '''
        right = super(WallFollowing,
                      self).sample_scan(scandata,
                                        270,
                                        window_size=WallFollowing.WINDOW)
        top = super(WallFollowing,
                    self).sample_scan(scandata,
                                      270 - WallFollowing.SAMPLE_ANGLE,
                                      window_size=WallFollowing.WINDOW)
        if self.started:
            return all((right <= WallFollowing.END_THRESHOLD,
                        top <= WallFollowing.END_THRESHOLD))
        else:
            return all((right <= WallFollowing.START_THRESHOLD,
                        top <= WallFollowing.START_THRESHOLD))

    def run(self, scandata):
        self.started = True

        right = super(WallFollowing,
                      self).sample_scan(scandata,
                                        270,
                                        window_size=WallFollowing.WINDOW)
        top = super(WallFollowing,
                    self).sample_scan(scandata,
                                      270 + WallFollowing.SAMPLE_ANGLE,
                                      window_size=WallFollowing.WINDOW)

        # This is what the top distance should be if the wall is straight and we
        # are parallel to it
        predicted_top = right / cos(radians(WallFollowing.SAMPLE_ANGLE))
        err = predicted_top - top

        # if positive, top sample is further away than expected, turn right.
        # Otherwise, turn left
        turnspeed = WallFollowing.TURN_GAIN * err

        # Slight nudge to stay about 1m from the wall
        turnspeed += -0.1 if right > WallFollowing.IDEAL_DIST else 0.1

        super(WallFollowing, self).move(turnspeed, WallFollowing.MOVE_SPEED)

    def clear_behaviour(self):
        self.started = False


class RobotControl():
    '''
    This class calls run on the other behaviours, whedn they have indicated
    that they should be run.
    '''

    def __init__(self):
        rospy.init_node('robot_control', anonymous=True)

        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.behaviours = [ObstacleAvoidance(self.vel_publisher),
                           WallFollowing(self.vel_publisher)]
        self.last_behaviour = None

    def start(self):
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.onscan)
        rospy.spin()

    def onscan(self, scandata):
        '''
        Callback for scan data, this is where we make any decisions.
        '''
        for behaviour in self.behaviours:
            if behaviour.shouldrun(scandata):
                behaviour.run(scandata)
                if self.last_behaviour and behaviour != self.last_behaviour:
                    print("Now running {}".format(behaviour.name))
                    self.last_behaviour.clear_behaviour()

                self.last_behaviour = behaviour
                return


if __name__ == "__main__":
    try:
        RobotControl().start()
    except rospy.ROSInterruptException:
        pass