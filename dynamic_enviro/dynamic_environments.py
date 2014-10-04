#!/usr/bin/env python

"""
October 3, 2014 - Adjustments were made to Paul's particle filter code 
to gather all the statements which interact with gazebo, rviz, or 
various rostopics.  The rest is a pseudo code structure.

October 4, 2014 - C+V: Finished formatting base code form Paul's inclass example (with some minor adjustments and renamings, and from pf_code that seemed useful, especially the transforms). Tested code - results was:

"""
#interfaces with ROS and Python
import rospy
#Getting topics from ROS
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
#Broadcaster/reciever information
import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
#Helper imports
import math
import time
import cv2 #openCV used to make window for publishing data
import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors

class TransformHelpers:
	""" Helper functions for making transformations of data from the RunMapping
	world to the script and back.  Will only be useful for us if we add an autonomy
	function to the robot """

	@staticmethod
	def convert_translation_rotation_to_pose(translation, rotation):
		""" Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
		return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

	@staticmethod
	def convert_pose_inverse_transform(pose):
		""" Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
		translation = np.zeros((4,1))
		translation[0] = -pose.position.x
		translation[1] = -pose.position.y
		translation[2] = -pose.position.z
		translation[3] = 1.0

		rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		euler_angle = euler_from_quaternion(rotation)
		rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))		# the angle is a yaw
		transformed_translation = rotation.dot(translation)

		translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
		rotation = quaternion_from_matrix(rotation)
		return (translation, rotation)

	@staticmethod
	def convert_pose_to_xy_and_theta(pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])


""" Difficulty Level 2 """
class RunMapping:
	""" Stores an occupancy field for an input map.  An occupancy field returns the distance to the closest
		obstacle for any coordinate in the map
		Attributes:
			map: the map to localize against (nav_msgs/OccupancyGrid)
			closest_occ: the distance for each entry in the OccupancyGrid to the closest obstacle
	"""

	def __init__(self, map):
		rospy.init_node("grid")	
		#create map properties, helps to make ratio calcs
		cv2.namedWindow("map")
		self.origin = [-10,-10]
		self.seq = 0
		self.resolution = 0.1
		self.n = 200
		#Giving initial hypotheses to the system
		self.p_occ = 0.5 #50-50 chance of being occupied
		self.odds_ratio_hit = 3.0 #this is arbitrary, can re-assign
		self.odds_ratio_miss = 0.2 #this is arbitrary, can reassign
		#calculates odds based upon hit to miss, equal odds to all grid squares
		self.odds_ratios = (self.p_occ)/(1-self.p_occ)*np.ones((self.n, self.n))
		
		#write laser pubs and subs
		rospy.Subscriber("scan", LaserScan, self.scan_received, queue_size=1)
		rospy.Publisher("map", OccupancyGrid)

		#note - in case robot autonomy is added back in
		tf_listener = TransformListener()
		tf_broadcaster = TransformBroadcaster()	

		# use super fast scikit learn nearest neighbor algorithm
		# nbrs = NearestNeighbors(n_neighbors=1,algorithm="ball_tree").fit(O)
		# distances, indices = nbrs.kneighbors(X)
		# self.closest_occ = {}
		# curr = 0
		# for i in range(self.map.info.width):
		# 	for j in range(self.map.info.height):
		# 		ind = i + j*self.map.info.width
		# 		self.closest_occ[ind] = distances[curr]*self.map.info.resolution
		# 		curr += 1
		

	def get_closest_obstacle_distance(self,x,y): #CHANGE TO get_closest_obstacle_path
		""" Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
			is out of the map boundaries, nan will be returned. """
			pass
			# x_coord = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
			# y_coord = int((y - self.map.info.origin.position.y)/self.map.info.resolution)
			# # check if we are in bounds
			# if x_coord > self.map.info.width or x_coord < 0:
			# 	return float('nan')
			# if y_coord > self.map.info.height or y_coord < 0:
			# 	return float('nan')
			# ind = x_coord + y_coord*self.map.info.width
			# if ind >= self.map.info.width*self.map.info.height or ind < 0:
			# 	return float('nan')
			# return self.closest_occ[ind]

	def is_in_map(self, x, y):
		"Returns boolean of whether or not a point is within map boundaries"
		#return if x is less than the origin, or larger than the map, ditto for y
		return (x < self.origin[0] or x > self.origin[0] + self.n*self.resolution or y<self.origin[1] or y>self.origin[1] + self.n*self.resolution)

	def scan_received(self, msg):
			""" Returns an occupancy grid to publish data to map"""	

		#make a pose stamp that relates to the odom of the robot
		p = PoseStamped(header=Header(stamp=msg.header.stamp,frame_id="base_link"), pose=Pose())
		self.odom_pose = self.tf_listener.transformPose("odom", p)
		# store the the odometry pose in a more convenient format (x,y,theta)
		self.odom_pose = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)

		for degree in range(360):
			if msg.ranges[degree] > 0.0 and msg.ranges[degree] < 5.0:
				#gets the position of the laser data points
				data_x = self.odom_pose[0] + msg.ranges[degree]*cos(degree*pi/180.0 + self.odom_pose[2])
				data_y = self.odom_pose[1] + msg.ranges[degree]*sin(degree*pi/180+self.odom[2])

				#maps laser data to a pixel in the map
				datax_pixel = int((data_x - self.origin[0])/self.resolution)
				datay_pixel = int((data_y - self.origin[1])/self.resolution)

				#maps the robot to a position
				robot = (data_x - self.odom_pose[0], data_y - self.odom_pose[1])

				#finds how far away the point is from the robot
				magnitude = sqrt(robot[0]**2 + robot[1]**2)

				#converts magnitude and robot position to pixels in the map
				n_steps = max([1, int(ceil(magnitude/self.resolution))])
				robot_step = (robot[0]/(n_steps-1), robot[1]/(n_steps-1))
				marked = set()

				for pixel in range(n_steps):
					curr_x = self.odom_pose[0] + pixel*robot_step[0]
					curr_y = self.odom_pose[1] + pixel*robot_step[1]
					if (self.is_in_map(curr_x, curr_y)):
						#make sure its in the map
						break

					x_ind = int((curr_x - self.origin[0])/self.resolution)
					y_ind = int((curr_y - self.origin[1])/self.resolution)
					if x_ind == datax_pixel and y_ind==datay_pixel:
						break
					if ((x_ind, y_ind) in marked):
						self.odds_ratios[x_ind, y_ind] *= self.p_occ / (1-self/p_occ) * self.odds_ratio_miss
						marked.add((x_ind, y_ind))
				if self.is_in_map(data_x, data_y):
					self.odds_ratios[datax_pixel, datay_pixel] *= self.p_occ/(1-self.p_occ) * self.odds_ratio_hit
		
		self.seq += 1
		if self.seq % 5 == 0:
			map = OccupancyGrid() #this is a nav msg class
			map.header.seq = self.seq
			map.header.stamp = msg.header.stamp
			map.header.frame_id = "map"
			map.info.origin.position.x = self.origin[0]
			map.info.origin.position.y = self.origin[1]
			map.info.width = self.n
			map.info.height = self.n
			map.info.resolution = self.resolution
			map.data = [0]*self.n**2 #the zero is a formatter, not a multiple of 0
			for i in range(self.n):
				#this is giving us the i,j grid square, occupancy grid
				for j in range(self.n):
					idx = i+self.n*j #makes horizontal rows (i is x, j is y)
					if self.odds_ratios[i,j] < 1/5.0:
						map.data[idx] = 0 #makes the gray
					elif self.odds_ratios[i,j] > 5.0:
						map.data[idx] = 100 #makes the black walls
					else:
						map.data[idx] = -1 #makes unknown
			self.pub.publish(map)

		image = np.zeros((self.odds_ratios.shape[0], self.odds_ratios.shape[1],3))
		#.shape() comes from being related to the np class
		for i in range(image.shape[0]):
			for j in range(image.shape[1]):
				if self.odds_ratios[i,j] < 1/5.0:
					image[i,j,:] = 1.0 #makes gray
				elif self.odds_ratios[i,j] > 5.0:
					image[i,j,:] = 0.0 #makes walls
				else:
					image[i,j,:] = 0.5 #none	

		x_odom_index = int((self.odom_pose[0] - self.origin[0])/self.resolution)
		y_odom_index = int((self.odom_pose[1] - self.origin[1])/self.resolution)

		#draw it!
		cv2.circle(image,(y_odom_index, x_odom_index), 2,(255, 0, 0))
		cv2.imshow("map", cv2.resize(image,(500,500)))
		cv2.waitKey(20) #effectively a delay

	def run(self):
		r = rospy.Rate(10)
		while not(rospy.is_shutdown()):
			r.sleep()


if __name__ == '__main__':
	try:
		n = RunMapping()
		n.run()
	except rospy.ROSInterruptException: pass