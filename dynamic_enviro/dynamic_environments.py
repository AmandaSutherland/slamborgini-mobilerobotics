#!/usr/bin/env python

"""
October 3, 2014 - Adjustments were made to Paul's particle filter code 
to gather all the statements which interact with gazebo, rviz, or 
various rostopics.  The rest is a pseudo code structure.

October 4, 2014 - C+V: 

"""
#interfaces with ROS and Python
import rospy
#Getting topics from ROS
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
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
		return not(x < self.origin[0] or x > self.origin[0] + self.n*self.resolution or y<self.origin[1] or y>self.origin[1] + self.n*self.resolution)

	def scan_received(self, msg):
			""" Returns an occupancy grid to publish data to map"""	

		#make a pose stamp that relates to the odom of the robot
		p = PoseStamped(header=Header(stamp=msg.header.stamp,frame_id="base_link"), pose=Pose())
		self.odom_pose = self.tf_listener.transformPose("odom", p)
		# store the the odometry pose in a more convenient format (x,y,theta)
		self.odom_pose = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)

		#get laser data
		#get position
		#get step along the way (so which time in terms of bayesian)
		#make sure data is in the map, you are in the map
		#update odds ratios
		#publish new ratios to the occupancy field




class ParticleFilter:
	""" The class that represents a Particle Filter ROS Node
		Attributes list:
			initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
			base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
			map_frame: the name of the map coordinate frame (should be "map" in most cases)
			odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
			scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
			n_particles: the number of particles in the filter
			d_thresh: the amount of linear movement before triggering a filter update
			a_thresh: the amount of angular movement before triggering a filter update
			laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
			pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
			particle_pub: a publisher for the particle cloud
			laser_subscriber: listens for new scan data on topic self.scan_topic
			tf_listener: listener for coordinate transforms
			tf_broadcaster: broadcaster for coordinate transforms
			particle_cloud: a list of particles representing a probability distribution over robot poses
			current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
								   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
			map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
	"""
	def __init__(self):
		self.initialized = False		# make sure we don't perform updates before everything is setup
		rospy.init_node('pf')			# tell roscore that we are creating a new node named "pf"

		self.base_frame = "base_link"	# the frame of the robot base
		self.map_frame = "map"			# the name of the map coordinate frame
		self.odom_frame = "odom"		# the name of the odometry coordinate frame
		self.scan_topic = "scan"		# the topic where we will get laser scans from 

		self.n_particles = 300			# the number of particles to use

		self.d_thresh = 0.2				# the amount of linear movement before performing an update
		self.a_thresh = math.pi/6		# the amount of angular movement before performing an update

		self.laser_max_distance = 2.0	# maximum penalty to assess in the likelihood field model

		# TODO3: define additional constants if needed

		#this seems pretty self explanatory. 

		""" May need to adjust thresh values if robot is to be still.  
		May need to reduce number of particles.
		Dynamic Variables vs. Static Variables, will these be different?
		"""

		# Setup pubs and subs

		# pose_listener responds to selection of a new approximate robot location (for instance using rviz)
		self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
		# publish the current particle cloud.  This enables viewing particles in rviz.
		self.particle_pub = rospy.Publisher("particlecloud", PoseArray)

		# laser_subscriber listens for data from the lidar
		

		self.particle_cloud = []

		self.current_odom_xy_theta = []

		# request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
		# TODO4: fill in the appropriate service call here.  The resultant map should be assigned be passed
		#		into the init method for OccupancyField
		""" Call the map """
		#rospy call to map "GetMap" imported from nav_msgs
		#set map as called map

		self.occupancy_field = OccupancyField(map)
		self.initialized = True

	def update_robot_pose(self):
		""" Update the estimate of the robot's pose given the updated particles.
			There are two logical methods for this:
				(1): compute the mean pose (level 2)
				(2): compute the most likely pose (i.e. the mode of the distribution) (level 1)
		"""
		# TODO: assign the lastest pose into self.robot_pose as a geometry_msgs.Pose object

		"""We need to update particle pose, not robot pose"""
		# first make sure that the particle weights are normalized
		self.normalize_particles()

	def map_calc_range(self,x,y,theta):
		""" Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
		# TODO6: nothing unless you want to try this alternate likelihood model
		""" us?"""

		pass



	@staticmethod
	def angle_normalize(z):
		""" convenience function to map an angle to the range [-pi,pi] """
		return math.atan2(math.sin(z), math.cos(z))

	@staticmethod
	def angle_diff(a, b):
		""" Calculates the difference between angle a and angle b (both should be in radians)
			the difference is always based on the closest rotation from angle a to angle b
			examples:
				angle_diff(.1,.2) -> -.1
				angle_diff(.1, 2*math.pi - .1) -> .2
				angle_diff(.1, .2+2*math.pi) -> -.1
		"""
		a = ParticleFilter.angle_normalize(a)
		b = ParticleFilter.angle_normalize(b)
		d1 = a-b
		d2 = 2*math.pi - math.fabs(d1)
		if d1 > 0:
			d2 *= -1.0
		if math.fabs(d1) < math.fabs(d2):
			return d1
		else:
			return d2

	@staticmethod
	def weighted_values(values, probabilities, size):
		""" Return a random sample of size elements form the set values with the specified probabilities
			values: the values to sample from (numpy.ndarray)
			probabilities: the probability of selecting each element in values (numpy.ndarray)
			size: the number of samples
		"""
		bins = np.add.accumulate(probabilities)
		return values[np.digitize(random_sample(size), bins)]

	def update_initial_pose(self, msg):
		""" Callback function to handle re-initializing the particle filter based on a pose estimate.
			These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
		xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(msg.pose.pose)
		self.initialize_particle_cloud(xy_theta)
		self.fix_map_to_odom_transform(msg)



	def fix_map_to_odom_transform(self, msg):
		""" Super tricky code to properly update map to odom transform... do not modify this... Difficulty level infinity. """
		(translation, rotation) = TransformHelpers.convert_pose_inverse_transform(self.robot_pose)
		p = PoseStamped(pose=TransformHelpers.convert_translation_rotation_to_pose(translation,rotation),header=Header(stamp=msg.header.stamp,frame_id=self.base_frame))
		self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
		(self.translation, self.rotation) = TransformHelpers.convert_pose_inverse_transform(self.odom_to_map.pose)

	def broadcast_last_transform(self):
		""" Make sure that we are always broadcasting the last map to odom transformation.
			This is necessary so things like move_base can work properly. """
		if not(hasattr(self,'translation') and hasattr(self,'rotation')):
			return
		self.tf_broadcaster.sendTransform(self.translation, self.rotation, rospy.get_rostime(), self.odom_frame, self.map_frame)

if __name__ == '__main__':
	n = RunMapping()
	r = rospy.Rate(5)

	while not(rospy.is_shutdown()):
		# in the main loop all we do is continuously broadcast the latest map to odom transform
		n.broadcast_last_transform()
		r.sleep()