#!/usr/bin/env python
# Citation: I am using the pa0_random_walk_fsm from class as a jumping off point!!!!

# Author: Kieran O'Day
# Date: 10/13/2021

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
        
        self.map = None
        self.old_map = None
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
        self.min_threshold_distance = min_threshold_distance
        self.wall_distance_goal = wall_distance_goal
        self.scan_angle_front = scan_angle_front
        self.scan_angle_right = scan_angle_right
        
        self.errors = []
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.next_id = 0
        
        self.last_callback = None # will set this in laser callback
        self.dt = None # will set this when we have some actual data
        # fsm variable.
        self._fsm = fsm.STOP
        
        self.odom_counter = 0
        
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
        print(robot_loc_map)
        # rotation_euler = tf.transformations.euler_from_quaternion(rot)
        
    def map_callback(self, msg):
        
        pass
        # print(len(msg.data))
        # self.old_map = self.map
        # self.map = Grid(msg.data, msg.info.width,
        #     msg.info.height, msg.info.resolution)
        # if self.detect_change():
        #     self._fsm = fsm.PD_CALC_CHANGE
        
    # detect change between our old map and our new map
    def detect_change(self):
        # set self.goal x and y location of the change
        return None

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
        
        if not self._fsm == fsm.PD_CALC_CHANGE or self._fsm == fsm.TAKING_PICTURE:
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
