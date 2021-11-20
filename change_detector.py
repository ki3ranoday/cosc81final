#!/usr/bin/env python
# Citation: Using Kieran O'Day's pa2 code from class as a jumping off point!!!!

# Author: Kieran O'Day, Nick Schoeller, Jonah Kershen, Ryan Alu
# Date: 11/15/2021

# Import of python modules.
import enum
import math  # use of pi.
import numpy as np
from Grid import Grid
import sys


# import of relevant libraries.
import rospy  # module for ROS APIs
import tf
from geometry_msgs.msg import Twist  # message type for cmd_vel
from sensor_msgs.msg import LaserScan  # message type for scan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import SetBool, SetBoolResponse
from skimage.measure import compare_ssim  # used for change detection
import cv2  # used for change detection


# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = "cmd_vel"

# Frequency at which the loop operates
FREQUENCY = 10  # Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = .2  # m/s
TURN_VELOCITY = math.pi / 8

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = .3 # m, threshold distance, should be smaller than range_max
WALL_DISTANCE_GOAL = .4 # m, the goal distance from the wall on the right

DEFAULT_SCAN_TOPIC = "scan" if len(sys.argv) > 1 and sys.argv[1] == 'scan' else 'base_scan'  # name of topic for Stage simulator. For Gazebo, 'scan'
# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD_FRONT = 135.0 / 180 * math.pi if len(sys.argv) > 1 and sys.argv[1] == 'scan' else -45.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD_FRONT = 225.0 / 180 * math.pi if len(sys.argv) > 1 and sys.argv[1] == 'scan' else +45.0 / 180 * math.pi

MIN_SCAN_ANGLE_RIGHT = 45.0 / 180 * math.pi if len(sys.argv) > 1 and sys.argv[1] == 'scan' else -45.0 / 180 * math.pi
MAX_SCAN_ANGLE_RIGHT = 135.0 / 180 * math.pi if len(sys.argv) > 1 and sys.argv[1] == 'scan' else -135.0 / 180 * math.pi


KP = 4.0
KI = 0.0
KD = 1.1


class fsm(enum.Enum):
    PD_CALC_CHANGE = 1 # used when we are trying to direct the robot to a change
    PD_CALC_WALL = 2 # used while the robot is following a wall
    MOVE = 3 # used to send commands for the robot to follow the wall
    TURN_CALC = 4 # used to 
    STOP = 5 # stops the robot
    TAKING_PICTURE = 6 # when the robot gets to a change it should enter taking picture state, then return to following wall
    TURN_TO_GOAL = 7
    GO_TO_GOAL = 8
    

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution):
        reshaped = np.reshape(occupancy_grid_data, (height, width))
        
        # generate numpy array representation of the grid to be used by cv2 for change detection
        self.grid = np.zeros([height,width], dtype=np.uint8)
        for row in range(height):
            for col in range(width):
                if reshaped[row, col] == 100:
                    self.grid[row, col] = 0
                elif reshaped[row, col] == 0:
                    self.grid[row, col] = 255
                else:
                    self.grid[row, col] = 100
        self.height = height
        self.width = width
        self.resolution = resolution


class ChangeDetector:
    def __init__(
        self,
        linear_velocity=0,
        angular_velocity=0,
        min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        wall_distance_goal = WALL_DISTANCE_GOAL,
        scan_angle_front=[MIN_SCAN_ANGLE_RAD_FRONT, MAX_SCAN_ANGLE_RAD_FRONT],
        scan_angle_right=[MIN_SCAN_ANGLE_RIGHT, MAX_SCAN_ANGLE_RIGHT],
        kp = KP, ki = KI, kd = KD
    ):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(
            DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1
        )
        
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        self.marker_pub = rospy.Publisher("markers", Marker, queue_size=1000) # get rid of queue size = 1 to make sure all the markers get published
        self._on_off_service = rospy.Service(
            "on_off", SetBool, self._turn_on_off_callback
        )
        self._odom_sub = rospy.Subscriber(
            'odom', Odometry, self._odom_callback)

        # Parameters.
        self.linear_velocity = linear_velocity  # Constant linear velocity set.
        self.angular_velocity = angular_velocity  # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance # min distance to the wall before turning around
        self.wall_distance_goal = wall_distance_goal # the distance from the right wall that the robot 
        self.scan_angle_front = scan_angle_front # the scan angles to consider in the front direction
        self.scan_angle_right = scan_angle_right # the scan angles to consider in the right direction
        
        self.errors = []
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.next_id = 0
        
        self.last_callback = None # will set this in laser callback
        self.dt = None # will set this when we have some actual data
        # fsm variable.
        self._fsm = fsm.STOP

        # change detection state variables
        self.start = True 
        self.map_start_x = None # starting x position, set in get_robot_pose
        self.map_start_y = None # starting y position, set in get_robot_pose
        
        self._is_back_at_start = False # flag to determine if robot has made a full loop
        self.first_map = None # map from first pass, set in map callback
        self.second_map = None # subsequent maps used to detect changes
        self.goal_x = None # x coordinate of detected change
        self.goal_y = None # y coordinate of detected change
        
        self.odom_counter = 0 # used to help establish when robot has made a full loop
        
        self.map_meta_data = None
        
        self.transform_listener = tf.TransformListener()
        
        while self.last_callback == None:
            rospy.sleep(.1) # wait for the laser callback to run before starting to move
    
    
    def get_robot_pose(self, msg):
        self.transform_listener.waitForTransform('map','odom', msg.header.stamp, rospy.Duration(0.1))
        odom_orientation = msg.pose.pose.orientation
        
        odom_position = msg.pose.pose.position
        (trans,rot) = self.transform_listener.lookupTransform('map','odom', msg.header.stamp)
        
        t = tf.transformations.translation_matrix(trans)
        r = tf.transformations.quaternion_matrix(rot)
        odomToMap = t.dot(r)
        
        robot_loc = np.array([odom_position.x, odom_position.y, 0.0, 1])
        robot_loc_map = odomToMap.dot(np.transpose(robot_loc))
        
        self.robot_x = robot_loc_map[0]
        self.robot_y = robot_loc_map[1]
        
        odom_orientation_euler = tf.transformations.euler_from_quaternion((odom_orientation.x, odom_orientation.y, odom_orientation.z, odom_orientation.w))
        rot_euler = tf.transformations.euler_from_quaternion(rot)
        
        self.robot_theta = odom_orientation_euler[2] + rot_euler[2]

        # check if the robot has made it back to its starting point
        if self.odom_counter > 50 and self.get_distance(robot_loc_map) <= 0.2:
            print("50")
            self._is_back_at_start = True

        # if this is the first odom callback, mark the starting position within map 
        # reference frame
        if self.start:
            print("Position", robot_loc_map)
            print("Shape", robot_loc_map.shape)
            self.map_start_x = robot_loc_map[0]
            self.map_start_y = robot_loc_map[1]
            self.start = False

    # checks the distance between the robot and where the robot started, to be used for map completion
    def get_distance(self, current_loc):
        dx = self.map_start_x - current_loc[0]
        dy = self.map_start_y - current_loc[1]

        return math.sqrt(dx * dx + dy * dy)

    # initializes Grid objects to be used for change detection
    def map_callback(self, msg):
        # initialize first map after full loop of the room
        if self._is_back_at_start and self.first_map is None:
            self.map_meta_data = msg.info
            self.first_map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        
        # once first map has been saved, check for changes each time
        elif self.first_map is not None:
            self.second_map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
            self.detect_change()
        
    # detect change between our old map and our new map
    # code adapted from https://www.pyimagesearch.com/2017/06/19/image-difference-with-opencv-and-python/
    def detect_change(self):
        first_map_blurred = cv2.GaussianBlur(self.first_map.grid,(7,7),0)
        second_map_blurred = cv2.GaussianBlur(self.second_map.grid,(7,7),0)

        (score, diff) = compare_ssim(first_map_blurred, second_map_blurred, full=True)
        diff = (diff * 255).astype("uint8")

        # threshold the difference image, followed by finding contours to
        # obtain the regions of the two input images that differ
        thresh = cv2.threshold(diff, 0, 255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        im2, cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            # compute the bounding box of the contour and then draw the
            # bounding box on both input images to represent where the two
            # images differ

            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(self.first_map.grid, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(self.second_map.grid, (x, y), (x + w, y + h), (0, 0, 255), 2)

            print("x: {}, y: {}, w: {}, h: {}".format(x, y, w, h))
            
            center_x = x + w/2
            center_y = y + h/2

            if w > .1/self.first_map.resolution and w < 1/self.first_map.resolution or h > .1/self.first_map.resolution and h < 1/self.first_map.resolution:
                print("Big Change")
                self.goal_x, self.goal_y = self.imageToMapFrame(center_x, center_y)
                x1,y1 = self.imageToMapFrame(x, y)
                x2,y2 = self.imageToMapFrame(x + w, y)
                x3,y3 = self.imageToMapFrame(x + w, y + h)
                x4,y4 = self.imageToMapFrame(x, y + h)
                # publish red markers to show the bounding box of the change
                self.publish_marker(x1, y1, r = 1)
                self.publish_marker(x2, y2, r = 1)
                self.publish_marker(x3, y3, r = 1)
                self.publish_marker(x4, y4, r = 1)
                # publish a green marker to show the middle of the change
                self.publish_marker(self.goal_x, self.goal_y, g = 1)
    
    # takes a cell in the image and converts it to a coordinate in the map frame
    def imageToMapFrame(self,x,y):
        origin_x = self.map_meta_data.origin.position.x
        origin_y = self.map_meta_data.origin.position.y
        
        new_x = origin_x + x * self.map_meta_data.resolution
        new_y = origin_y + y * self.map_meta_data.resolution
        
        return new_x, new_y
       
    # publishes a marker to the map 
    def publish_marker(self, x, y, r = 0, g = 0, b = 0):
        """Publishing an arrow at x, y facing theta"""
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.CYLINDER
        marker_msg.id = self.next_id
        self.next_id += 1
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.color.r = r
        marker_msg.color.g = g
        marker_msg.color.b = b
        marker_msg.color.a = 1
        marker_msg.scale.x = .1
        marker_msg.scale.y = .1
        marker_msg.scale.z = .1
        self.marker_pub.publish(marker_msg)
    
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

    def _turn_on_off_callback(self, req):
        resp = SetBoolResponse()
        if not req.data:
            self._fsm = fsm.STOP
            self.stop()
            resp.success = True
            resp.message = "Robot stopped"
        else:
            if self._fsm == fsm.STOP:
                self._fsm = fsm.MOVE
                resp.success = True
                resp.message = "Robot activated"
            else:
                resp.success = False
                resp.message = "Robot already moving"

        return resp   
    
    def get_ranges(self, angle_a, angle_b, msg):
        ranges = []
        for i,range in enumerate(msg.ranges):
            range_angle = msg.angle_min + msg.angle_increment * i
            while range_angle < 0:
                range_angle += 2 * math.pi
                
            while angle_a < 0:
                angle_a += 2 * math.pi
            
            while angle_b < 0:
                angle_b += 2 * math.pi
                
            # print('Range angle %s, angle_a %s, angle_b %s' %(range_angle, angle_a, angle_b))

            if range_angle > min(angle_a, angle_b) and range_angle < max(angle_a, angle_b):
                ranges.append(range)
                
        return ranges
    
    def _odom_callback(self, msg):
        if self.odom_counter % 10 == 0:
            self.get_robot_pose(msg)
        self.odom_counter += 1

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.

        # Find the minimum range value between min_scan_angle and
        # max_scan_angle
        # If the minimum range value is closer to min_threshold_distance, change the fsm
        # Note: You have to find the min index and max index.
        # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        
        if self.last_callback: # get the time and update the dt
            self.dt = rospy.get_rostime() - self.last_callback
        self.last_callback = rospy.get_rostime()
        
        error = self.wall_distance_goal - min(self.get_ranges(self.scan_angle_right[0], self.scan_angle_right[1], msg))
        self.errors.append(error)
        
        if not (self._fsm == fsm.TURN_TO_GOAL or self._fsm == fsm.GO_TO_GOAL or self._fsm == fsm.TAKING_PICTURE):
            if (
                np.min(self.get_ranges(self.scan_angle_front[0], self.scan_angle_front[1], msg))
                < self.min_threshold_distance
            ):
                self._fsm = fsm.TURN_CALC # if theres something in front of the robot, it will turn left
            else:
                self._fsm = fsm.PD_CALC_WALL # otherwise it will try to follow the right wall

    ####### ANSWER CODE END #######

    def spin(self):
        rate = rospy.Rate(FREQUENCY)  # loop at 10 Hz.
        rotation_started = False
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            # If the flag fsm is set to MOVE_FORWARD, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.
            # print(self._fsm)
            if self.goal_x != None and self.goal_y != None:
                self._fsm = fsm.TURN_TO_GOAL
            
            # states for following wall
            if self._fsm == fsm.PD_CALC_WALL:
                if len(self.errors) > 0:
                    error = self.errors[-1]
                    # print("Error is: %s" % error)
                    self.linear_velocity = min(LINEAR_VELOCITY, LINEAR_VELOCITY * (1-min(abs(self.errors[-1]), 1)))
                    self.angular_velocity = self.kp * error
                    if len(self.errors) > 1 and self.dt:
                        self.angular_velocity += self.kd * ((self.errors[-1] - self.errors[-2]) / self.dt.to_sec())
                        self.angular_velocity += self.ki * (self.dt.to_sec() * sum(self.errors))
                    # print("Setting angular velocity to %s deg/s (%s rad/s)"%(self.angular_velocity * 180 / math.pi, self.angular_velocity))
                    self._fsm = fsm.MOVE
            if self._fsm == fsm.TURN_CALC:
                # self.angular_velocity = TURN_VELOCITY if self.angular_velocity == 0 else np.sign(self.angular_velocity) * TURN_VELOCITY
                self.angular_velocity = TURN_VELOCITY
                self.linear_velocity = 0
                self._fsm = fsm.MOVE
            if self._fsm == fsm.TURN_TO_GOAL:
                goal_theta = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
                angle = goal_theta - self.robot_theta
                
                if abs(angle) > math.pi:
                    angle = angle - np.sign(angle) * 2 * math.pi
                    
                if abs(angle) > math.pi/8:
                    self.linear_velocity = 0
                    self.angular_velocity = TURN_VELOCITY * np.sign(angle)
                    self._fsm = fsm.MOVE
                else:
                    self._fsm = fsm.GO_TO_GOAL
            if self._fsm == fsm.GO_TO_GOAL:
                goal_dist = math.sqrt((self.goal_x - self.robot_x) ** 2 + (self.goal_y - self.robot_y) ** 2)
                if goal_dist > .5:
                    self.linear_velocity = LINEAR_VELOCITY
                    self.angular_velocity = 0
                    self._fsm = fsm.MOVE
                else:
                    self._fsm = fsm.TAKING_PICTURE
            if self._fsm == fsm.TAKING_PICTURE:
                # We ended up not actually taking a picture, just going to the change position and stopping there,
                # We chose not to take a picture because the change detector worked best for removals of objects, not additions
                # so taking a picture of nothing doesn't make much sense
                self._fsm = fsm.STOP
            if self._fsm == fsm.MOVE:
                self.move(self.linear_velocity, self.angular_velocity)
            if self._fsm == fsm.STOP:
                self.angular_velocity = 0
                self.linear_velocity = 0

            ####### ANSWER CODE END #######

            rate.sleep()


def main():
    
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("detect_change")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    detector = ChangeDetector()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(detector.stop)

    # Robot random walks.
    try:
        detector.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
