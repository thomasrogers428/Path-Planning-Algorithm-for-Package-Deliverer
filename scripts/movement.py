#!/usr/bin/env python

# Brandon Feng, Seungjae Lee, Thomas Rogers
#assumption is that robot is initialized facing in the direction of the x axis


from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from node import Node
from search import Search

import numpy
import rospy
import math
import tf

FREQUENCY = 100 #Frequency of Message Publishing
VELOCITY = 2.5 #m/s, init velocity for move_straight 
VELOCITY2 = 0.5 #m/s, init velocity constant for obstacle_move, obstacle_turn states
ERROR1 = 0.05 #Error bound for point to point
ERROR2 = 0.02 #Error bound for point to line
ANG_ERROR = 0.05*numpy.pi/180 # angular error bound for angle difference
ANG_VEL =numpy.pi/8 # angular velocity for obstacle_turn state
K_P = 6 #potential gain
K_D = 2	#derivative gain
SET_DISTANCE = 1 #distance from wall in meters
OBSTACLE_DIST = SET_DISTANCE*1.2 #m moves away from obstacle if it's within given distance away



class Movement:
	'''class that implements the PD controller and Bug2 algorithm for local path planning'''
	def __init__(self):
		rospy.init_node("movement_node")


		self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laserscan_callback)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback)

		self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		self.initialized_x = 0
		self.initialized_y =  0

		self.path = None

		self.right_distance = 0
		self.center_distance = 0

		self.curr_x = 0
		self.curr_y = 0
		self.curr_r = 0

		#store current time and time of previous run of loop in main (will be used for calculating delta_t (timeframe))
		self.curr_time = 0
		self.prev_time = 0

		#store difference between SET_DISTANCE and right_distance for current time and previous time
		self.error = 0
		self.prev_error = 0

		#value that is calculated and passed as the angular velocity when there is no obstruction
		self.U_t = 0

		#state for finite machine. decides the action of the robot
		self.state = "rotate"

		rospy.sleep(2)

	def laserscan_callback(self, laserscan_msg):
		'''laser callback function. Stores the right distance and center distance for obstacle avoidance'''
		temp_min_right = 20

		temp_min_center = 20

		for i in range(len(laserscan_msg.ranges)):

			angle = laserscan_msg.angle_min + i * laserscan_msg.angle_increment

			if  (-numpy.pi/4 - .5) < angle < (-numpy.pi/4 + .5):
				if laserscan_msg.ranges[i] < temp_min_right:
					temp_min_right = laserscan_msg.ranges[i]

			if (laserscan_msg.angle_increment*-20) < angle < (laserscan_msg.angle_increment*20):
				if laserscan_msg.ranges[i] < temp_min_center:
					temp_min_center = laserscan_msg.ranges[i]


		self.right_distance = temp_min_right
		self.center_distance = temp_min_center

	def odometry_callback(self, odometry_msg):
		# odometry callback, stores the position(x,y yaw) of the robot.

		self.curr_x = odometry_msg.pose.pose.position.x + self.initialized_x
		self.curr_y = odometry_msg.pose.pose.position.y + self.initialized_y
		quat_robot = odometry_msg.pose.pose.orientation
        	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion ([quat_robot.x, quat_robot.y, quat_robot.z, quat_robot.w])
		self.curr_r = yaw

	def create_path(self):
		'''finds an optimal path based on given packages and stores the path.'''

		robot_capacity = 5
		init_robot_location = (self.curr_x, self.curr_y)
		#package_info = {} #simple test with minimal obstacle avoidance
		#package_info[1] = ((2,1), (4,4), 1)
		#package_info[2] = ((2,1), (9,0), 2)
		#package_info[3] = ((2,1), (9,0), 3)
		#package_info[4] = ((6,5), (-15,3), 4)
		#package_info[5] = ((6,5), (4,4), 5)
		
		# Test for experiment. has obstacle avoidance
		package_info = {}
		package_info[1] = ((16,10), (4,4), 1)
		package_info[2] = ((16,10), (9,0), 2)
		package_info[3] = ((4,5), (-16,0), 3)
		package_info[4] = ((4,5), (5,-15), 4)
		
		# Test case for showing limitation of Bug2
		#package_info = {}
		#package_info[1] = ((0,0), (14,17), 1)

		# Search Algorithm for optimal path
		search = Search(robot_capacity, init_robot_location, package_info)
		self.path = search.search()[0]
		
		# Prints the optimal path(for debugging)
		prev_loc = None
		for node in self.path:
			if prev_loc != node.get_robot_location():
				print(node.get_robot_location())
				prev_loc = node.get_robot_location()
	def find_line_angle(self, start, end):
		'''returns the angle of two points in range of (-pi, pi)'''
		start_x = start[0]
		end_x = end[0]
		start_y = start[1]
		end_y = end[1]

		if end_x - start_x != 0:
			rotation = math.atan2((end_y- start_y),(end_x - start_x))
		else:
			rotation = 0.0
		return rotation




	def distance_to_line(self, p0, p1, p2):
    		'''
		p0 is the current position
    		p1 and p2 points define the line
		returns the distance from p0 and line formed by p1 and p2
		'''
    		up_eq = math.fabs((p2[1] - p1[1]) * p0[0] - (p2[0] - p1[0]) * p0[1] + (p2[0] * p1[1]) - (p2[1] * p1[0]))
    		lo_eq = math.sqrt(pow(p2[1] - p1[1], 2) + pow(p2[0] - p1[0], 2))
    		return up_eq / lo_eq

	def distance_to_point(self, p1, p2):
		'''
		p1 and p2 are (x,y) points
		returns the distance of p1 and p2
		'''
    		return math.sqrt(pow(p2[1] - p1[1], 2) + pow(p2[0] - p1[0], 2))

	def angle_to_2pi(self,angle):
		'''given angle from -pi to pi, returns it as a value of 0 to pi'''
		if 0<= angle <= numpy.pi:
			return angle
		else:
			return 2*numpy.pi + angle

	def main(self):
		''' publishes velocity based on bug2 algorithm and PD controller.'''
		# set up frequency of publishing a message.
		rate = rospy.Rate(FREQUENCY)
		
		# set up velocity msg
		vel_msg = Twist()

		# initialization of variables
		i = 0 # used to update start and end locations 
		start_location = self.path[0].get_robot_location()
		goal_location = self.path[1].get_robot_location()
		dist_sl_to_gl = self.distance_to_point(start_location,goal_location)
		goal_r = self.find_line_angle(start_location, goal_location)
		#print start_location, goal_location


		while not rospy.is_shutdown():
			if i < len(self.path)-1:
				# still has a path to visit
				dist_cl_to_gl = self.distance_to_point((self.curr_x, self.curr_y),goal_location)
				
				# Finite State Machine
				if self.state == "rotate" and abs(goal_r - self.curr_r) <= ANG_ERROR:
					self.state = "move_straight"
					#print "moving straight"

				elif self.state == "obstacle_move" and self.distance_to_line((self.curr_x, self.curr_y), start_location,goal_location)<= ERROR2:
					self.state = "rotate"
					#print "rotating"

				elif self.state == "move_straight" and self.center_distance <= OBSTACLE_DIST:
					self.state = "obstacle_turn"
					#print "preparing wall follow"
				elif self.state == "obstacle_turn" and self.distance_to_line((self.curr_x, self.curr_y), start_location,goal_location) > ERROR2:
					self.state = "obstacle_move"
					#print "wall follow"





				# updating vel_msg based on the current state
				if self.state == "move_straight":
					#facing goal, moving towards the goal
					if dist_sl_to_gl != 0:
						vel_msg.linear.x = VELOCITY*dist_cl_to_gl/dist_sl_to_gl
					else:
						vel_msg.linear.x = 0
					vel_msg.angular.z = 0

				elif self.state == "rotate":
					#not pointing towards goal, need to rotate towards the goal
					ang_diff = self.angle_to_2pi(goal_r) - self.angle_to_2pi(self.curr_r)
					if ang_diff <= numpy.pi:
						vel_msg.angular.z = ang_diff
					else:
						vel_msg.angular.z = (ang_diff - 2*numpy.pi)
					vel_msg.linear.x = 0
					


				else:
					self.error = SET_DISTANCE - self.right_distance

					if self.state == "obstacle_turn":
						# following the left side of the wall
						# preliminary stage for obstacle avoidance
						vel_msg.angular.z = ANG_VEL
						vel_msg.linear.x = VELOCITY2
					else:
						# BUG2: follows wall using PD controller
						time_frame = rospy.get_rostime() -self.prev_time
						time_frame = time_frame.to_sec()
						if time_frame == 0:
							time_frame = 0.01

						# set up angular velocity using PD controller
						self.U_t = (K_P*self.error)+ K_D*((self.error - self.prev_error)/time_frame)
						vel_msg.angular.z = self.U_t
						
						# set up linear velocity
						dist_to_line = self.distance_to_line((self.curr_x, self.curr_y), start_location,goal_location)
						if dist_to_line < 1:
							vel_msg.linear.x = VELOCITY2*dist_to_line
						else:
							vel_msg.linear.x = VELOCITY2

					self.prev_error = self.error # update error

				self.prev_time = rospy.get_rostime() # update time
				
				self.velocity_pub.publish(vel_msg) # publish velocity

				if dist_cl_to_gl <= ERROR1:
					# reached the goal, update new start and goal location and other related variables
					self.state = "rotate" # update state

					# update variables
					i += 1
					start_location = (self.curr_x, self.curr_y)
					goal_location = self.path[i+1].get_robot_location()
					dist_sl_to_gl = self.distance_to_point(start_location,goal_location)
					goal_r = self.find_line_angle((self.curr_x, self.curr_y), goal_location)
			rate.sleep()

if __name__ == "__main__":
	m = Movement()
	m.create_path() # create optimal path using Dijkstra
	m.main() # run BUG2 for local path planning

