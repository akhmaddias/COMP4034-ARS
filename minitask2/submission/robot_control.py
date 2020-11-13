#!/usr/bin/env python
import rospy
import tf
import random

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
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
        super(ObstacleAvoidance, self).__init__(vel_pub)  # Python2 is bad
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


class RandomWalk(RobotBehaviour):
    '''
    This behaviour will choose a target theta, turn to it and then drive
    forwards for 3m. It uses proportional contorl (using odometry data) for
    both the turning and driving.
    '''

    def __init__(self, vel_pub):
        super(RandomWalk, self).__init__(vel_pub)  # Python2 is bad
        self.name = "Random Walk"

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

        self.clear_behaviour()

        self.x = None
        self.y = None
        self.theta = None

    def shouldrun(self, scandata):
        '''
        Always run random walk as its the last behaviour if others are not
        appropriate.
        '''
        return True

    def run(self, scandata):
        if not all((self.x, self.y, self.theta)):
            super(RandomWalk, self).forward(0.01)
            return  # Move a little bit if we don't have any odometry data yet

        if not self.target_angle:
            self.target_angle = random.uniform(0, PI * 2)
            print("Turning to heading: {0} degrees".format(
                int(degrees(self.target_angle))))

        # Work out the error if turning left and if turning right, take the
        # smaller value. This prevents us from ever turning more than 180 deg
        lh_err = (self.theta - self.target_angle) % (2 * PI)
        rh_err = (self.target_angle - self.theta) % (2 * PI)
        turn_err = min(lh_err, rh_err)

        move_err = THREE_METERS - self.progress

        if self.turning:
            if turn_err <= TURNING_ERROR_THRESHOLD:
                print("Done turning")
                self.turning = False
                return

            # turnspeed is proportional and clamped between min and max turn speed
            turnspeed = min(max(turn_err * TURN_GAIN,
                                TURN_MIN_SPEED),
                            TURN_MAX_SPEED)
            turnspeed = turnspeed if rh_err < lh_err else -turnspeed
            # print("Turnspeed = {}".format(turnspeed))
            super(RandomWalk, self).turn(turnspeed)

        else:
            if turn_err > TURNING_ERROR_THRESHOLD * 2:
                print("Robot has drifted, turning again")
                self.turning = True
                return  # If we've strayed from the target angle, start turning again

            if move_err > 0:
                forward_speed = min(max(move_err * FORWARD_GAIN, FORWARD_MIN_SPEED),
                                    FORWARD_MAX_SPEED)
                # print("Forward speed = {}".format(forward_speed))
                super(RandomWalk, self).forward(forward_speed)
            else:
                print("Done driving forward")
                self.clear_behaviour()

    def clear_behaviour(self):
        self.turning = True
        self.progress = 0
        self.target_angle = None

    def odom_cb(self, odom):
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        if not self.turning:
            self.progress += sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        self.x, self.y = x, y

        quart = [odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quart)
        self.theta = yaw


class RobotControl():
    '''
    This class calls run on the other behaviours, whedn they have indicated
    that they should be run.
    '''

    def __init__(self):
        rospy.init_node('robot_control', anonymous=True)

        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.behaviours = [ObstacleAvoidance(self.vel_publisher),
                           WallFollowing(self.vel_publisher),
                           RandomWalk(self.vel_publisher)]
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
