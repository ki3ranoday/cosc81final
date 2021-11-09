#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Kieran O'Day
# Date: 10/03/2021
# Citation: Basing a lot of the initial setup on pa1_mobile_motion.py

# import of relevant libraries
import math
import numpy
import rospy  # module for ROS APIs
import tf
from Grid import Grid
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid  # message type for odom
from sensor_msgs.msg import LaserScan  # message type for scan


# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_SCAN_TOPIC = '/scan'  # for running on real robot
# DEFAULT_SCAN_TOPIC = 'base_scan' # for running in simulation
DEFAULT_GRID_TOPIC = 'map'


FREQUENCY = 10
ANGULAR_VELOCITY = math.pi/8  # rad/s
LINEAR_VELOCITY = .2  # m/s

MAP_WIDTH = 50  # meters
MAP_HEIGHT = 50  # meters
MAP_RESOLUTION = .1  # meters


def degreeFromRad(rad):
    return rad * 180/math.pi


class StateEstimator():
    def __init__(self):
        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(
            DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        self._odom_sub = rospy.Subscriber(
            DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback)

        self.map_sub = rospy.Subscriber(
            "map", OccupancyGrid, self.map_callback, queue_size=1)

        self._grid_pub = rospy.Publisher(
            DEFAULT_GRID_TOPIC, OccupancyGrid,  queue_size=1000)

        self._laser_sub = rospy.Subscriber(
            DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        self.transformer = tf.TransformListener()

        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.current_dist_front = None

        self.move_for = 0    # how long repeat the current message for
        self.linear_vel = 0  # current linear velocity
        self.angular_vel = 0  # current angular velocity

        grid_width = int(MAP_WIDTH/MAP_RESOLUTION)
        grid_height = int(MAP_HEIGHT/MAP_RESOLUTION)
        self.grid = Grid([-1 for i in range(grid_width * grid_height)],
                         grid_width, grid_height, MAP_RESOLUTION)

        # before you start before you start anything, wait for the odom callback to figure out where the robot is
        while self.current_theta == None or self.current_x == None or self.current_y == None:
            print('waiting')
            rospy.sleep(.1)

    def map_callback(self, msg):
        self.map = Grid(msg.data, msg.info.width,
            msg.info.height, msg.info.resolution)

    def _odom_callback(self, msg):
        o = msg.pose.pose.orientation
        o_euler = tf.transformations.euler_from_quaternion(
            (o.x, o.y, o.z, o.w))

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = o_euler[2]

        # print("odom: ", self.current_x, self.current_y, self.current_theta)

    # given an angle, gives the index in the ranges array (need to round to int outside depending on if you want to go up or down)
    # angle = angle_min + i * ang_inc
    def angleToRangeIndex(self, angle, angle_min, angle_inc):
        return (angle - angle_min)/angle_inc

    def _laser_callback(self, msg):

        zeroIndex = int(self.angleToRangeIndex(
            0, msg.angle_min, msg.angle_increment))
        self.current_dist_front = msg.ranges[zeroIndex]

        for i, range in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            scanTime = msg.header.stamp + \
                rospy.Duration(i * msg.time_increment)
            scan_frame = msg.header.frame_id

            # calculate the x and y in the laser frame
            x, y = numpy.cos(angle) * range, numpy.sin(angle) * range

            # find the transformation between the laser frame and the odom frame at the time of the scan
            self.transformer.waitForTransform(
                '/odom', scan_frame, scanTime, rospy.Duration(5))
            trans, rot = self.transformer.lookupTransform(
                '/odom', scan_frame, scanTime)
            theta = tf.transformations.euler_from_quaternion(rot)[2]

            # make a transformation matrix
            laserToOdom = numpy.matrix([
                [numpy.cos(theta), -numpy.sin(theta), 0, trans[0]],
                [numpy.sin(theta), numpy.cos(theta), 0, trans[1]],
                [0, 0, 1, trans[2]],
                [0, 0, 0, 1]
            ])

            # transform the point from the laser scan into odom
            p_laser_matrix = numpy.matrix([[x], [y], [0], [1]])
            p_odom_matrix = laserToOdom * p_laser_matrix

            if not (
                p_odom_matrix[0, 0] is None or p_odom_matrix[1, 0] is None or
                math.isinf(p_odom_matrix[0, 0]) or math.isnan(p_odom_matrix[0, 0]) or
                math.isinf(p_odom_matrix[1, 0]) or math.isnan(
                    p_odom_matrix[1, 0])
            ):
                # map the obstacle on the grid
                if range < msg.range_max:
                    self.map_obstacle(
                        self.current_x, self.current_y, p_odom_matrix[0, 0], p_odom_matrix[1, 0])
                else:
                    self.map_free_space(
                        self.current_x, self.current_y, p_odom_matrix[0, 0], p_odom_matrix[1, 0])
        # publish the occupancy grid message
        self.publish_grid()

    def publish_grid(self):
        msg = OccupancyGrid()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = '/odom'
        msg.info.resolution = MAP_RESOLUTION
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = - MAP_WIDTH/2
        msg.info.origin.position.y = - MAP_HEIGHT/2

        msg.data = self.grid.as_list()

        self._grid_pub.publish(msg)

    # map the obstacle, points are given in the odom reference frame
    # I am going to say the center of my map is 0,0 in the odom frame,
    def map_obstacle(self, c_x, c_y, x, y):
        c_grid_x, c_grid_y = self.odom_to_grid(c_x, c_y)
        grid_x, grid_y = self.odom_to_grid(x, y)

        if grid_x != None and grid_y != None and c_x != None and c_y != None:
            # map the line between cx and cy as free space
            self.map_free_space(c_grid_x, c_grid_y, grid_x, grid_y)
            if grid_x >= 0 and grid_x < self.grid.width and grid_y >= 0 and grid_y < self.grid.height:
                self.grid.set(grid_x, grid_y, 100)  # map the obstacle

    # I was very confused by the Bresenham algorithm, and stumbled upon python code for it when I was researching
    # so I didn't want to plagiarize that code, and wanted to implement something I actually understanded
    # I found this website https://www.redblobgames.com/grids/line-drawing.html explaining a way to do it with linear interpolation
    # I used this website as a basis for undestanding what the algorithm was doing (it explains it fantastically)
    # and that cite led me to the DDA algorithm
    # I adapted the code for this from the wikipedia article on it where it was coded in c++
    # (and changed that code so it made more sense to me)
    # CITATION: https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)

    def map_free_space(self, c_x, c_y, x, y):
        dx = x - c_x
        dy = y - c_y
        # either the dx per step or dy per step will be 1 or -1, (depending on which distance is longer)
        # then the other one will be <= 1 and >= -1, so as we step down the line, some adjacent square will be filled in
        num_steps = abs(dx) if abs(dx) > abs(dy) else abs(dy)
        dx_per_step = float(dx) / num_steps
        dy_per_step = float(dy) / num_steps
        for step in range(int(num_steps)):
            point_x = int(c_x + dx_per_step * step)
            point_y = int(c_y + dy_per_step * step)
            if point_x >= 0 and point_x < self.grid.width and point_y >= 0 and point_y < self.grid.height:
                # don't get rid of found obstacles # this is bad if obstacles move around, but good if the robot is mapping a static environment.
                if self.grid.cell_at(point_x, point_y) != 100:
                    self.grid.set(point_x, point_y, 0)

    # my grid is going to be centered at 0,0 in the odom frame, so we need to translate any point
    # from the odom frame up and right by half the map height and half the map width

    def odom_to_grid(self, x, y):
        grid_x = int((x + MAP_WIDTH/2)/MAP_RESOLUTION)
        grid_y = int((y + MAP_HEIGHT/2)/MAP_RESOLUTION)
        return grid_x, grid_y  # not necessarily on the grid

    def moveForDuration(self):
        rate = rospy.Rate(FREQUENCY)
        duration = rospy.Duration(self.move_for)
        start_time = rospy.get_rostime()
        print("Move at %s m/s and %s rad/s for %s seconds" %
              (self.linear_vel, degreeFromRad(self.angular_vel), self.move_for))

        while rospy.get_rostime() - start_time < duration:
            self.move(self.linear_vel, self.angular_vel)
            rate.sleep()

    # very similar to move for duration, except using a tick count lets me slightly alter the angular
    # velocity so that the robot rotates more closely to the target angle
    def moveForTickCount(self):
        rate = rospy.Rate(FREQUENCY)
        print("Move at %s m/s and %s rad/s for %s ticks" %
              (self.linear_vel, degreeFromRad(self.angular_vel), self.tick_count))

        while self.tick_count > 0:
            self.move(self.linear_vel, self.angular_vel)
            self.tick_count -= 1
            rate.sleep()

    def straight_line(self, distance):
        while self.current_dist_front == None or math.isinf(self.current_dist_front) or math.isnan(self.current_dist_front):
            print('waiting for laser reading')
            rospy.sleep(.1)
        startDist = self.current_dist_front
        startX, startY = self.current_x, self.current_y
        # find the number of ticks that we need to move straight for, and round to an even integer
        self.tick_count = round(
            abs(((distance * FREQUENCY)/LINEAR_VELOCITY)), 0)
        if(self.tick_count > 0):
            self.linear_vel = abs(distance * (FREQUENCY)/self.tick_count)
            self.angular_vel = 0

            self.moveForTickCount()

        # wait a split second for the laser and odom msgs to definitely come in
        rospy.sleep(.1)
        laserEst = self.current_dist_front - startDist
        odomEst = math.sqrt((self.current_x-startX) ** 2 +
                            (self.current_y - startY) ** 2)
        print("Rosbot commanded to go %sm, laser readings estimate %sm, odom estimates %sm" % (
            distance, laserEst, odomEst))
        print("%s, %s, %s" % (distance, laserEst, odomEst))

    # takes degrees by default, but converts to radians

    def rotate_rel(self, angle, in_radians=False):
        while self.current_dist_front == None or math.isinf(self.current_dist_front) or math.isnan(self.current_dist_front):
            print('waiting for laser reading')
            rospy.sleep(.1)
        startDist = self.current_dist_front
        startTheta = self.current_theta
        if not in_radians:
            angle = angle * math.pi/180
        # set spin direction to the correct spin direction
        spin_direction = 1 if angle > 0 else -1
        # find the number of ticks that we need to rotate for, and round to an even integer
        self.tick_count = round(abs(((angle * FREQUENCY)/ANGULAR_VELOCITY)), 0)
        if(self.tick_count > 0):
            self.linear_vel = 0
            # then calculate the angular velocity based on that tick count, since we rounded, it will be
            # very slightly more or less than the ANGULAR VELOCITY constant
            self.angular_vel = spin_direction * \
                abs(angle*(FREQUENCY)/self.tick_count)

            self.moveForTickCount()

        # wait a split second for the laser and odom msgs to definitely come in
        rospy.sleep(.1)
        # start dist is the distance to the wall which makes a right angle with the wall, then end dist is hypotnuse of that triangle
        print(startDist, self.current_dist_front)
        laserEst = numpy.arccos(
            startDist/self.current_dist_front) * 180/math.pi
        odomEst = (self.current_theta - startTheta) * 180/math.pi
        print("Rosbot commanded to turn %s, laser readings estimate %s, odom estimates %s" % (
            angle*180/math.pi, laserEst, odomEst))
        print("%s,%s,%s" % (angle*180/math.pi, laserEst, odomEst))

    # rotates to a goal angle in the odom frame
    def rotate_abs(self, goal_theta, in_radians=False):
        if not in_radians:  # the function by default takes in degrees but needs radians to work
            goal_theta = goal_theta * math.pi/180

        rel_ang = goal_theta - self.current_theta
        print(rel_ang)
        self.rotate_rel(rel_ang, True)

    def square(self, size):
        square_angles = [90, 180, 270, 0]
        for i in range(4):
            self.straight_line(size)
            self.rotate_abs(square_angles[i])

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)


def main():
    """Main function."""
    # 1st. initialization of node.
    rospy.init_node("state_estimator")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    state_estimator = StateEstimator()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(state_estimator.stop)

    try:
        # tests here
        rospy.sleep(2)  # wait to make sure laser readings start coming in
        # state_estimator.straight_line(1)
        state_estimator.rotate_rel(30)
        # state_estimator.square(1)
        rospy.spin()  # keep runnning the callbacks from the bot

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
