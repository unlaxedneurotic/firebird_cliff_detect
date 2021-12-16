#!/usr/bin/env python
####################################
# Author:
# Saransh Jindal
# jindalsaransh@gmail.com
# using ROS Indigo on FireBird VI by NexRobotics
#######################################################
# This script will subscribe to the following topics:
# 1. arduino_dist - data from arduino ultrasonic sensor that reads dist to the floor
# 2. ros0xrobot/sonar - data from the firebird inbuilt sonars
# 3. ros0xrobot/mqtt_vel - data from mqtt server that will be served by mqtt_conv
# 
# The sonar sensors are indexed as follows:
# 	view from top - values start with the leftmost sensor and follows clockwise
# 
# 							       _2_
# 							   1 /	   \ 3
# 							  0 |	    | 4
# 							   7 \ _ _ / 5
# 								    6
# The indexes for movement and direction start from the front
# and follow clockwise[stop is 0]:
# 			8 1 2
# 			7 0 3 
# 			6 5 4
# 
# After checking for obstacles or cliffs in the direction of the robot's movement,
# we take decision on whether we wish to let the robot move or not, or in the case it is moving, to stop it.
# The twist message is published on the topic `ros0xrobot/cmd_vel`.
# 
# 
# 
# 
# 
######################################################

import rospy
from rospy.topics import Publisher, Subscriber
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist



global pub
pub = None
twist_zero_msg = Twist()

class Server:

	#checklist for each direction, enter the index of sensor we wish to check
	sensor_check_list = {
		1:[1,3],
		2:[3,4],
		3:[3,4,5],
		4:[4,5,6],
		5:[5,6,7],
		6:[6,7,0],
		7:[7,0,1],
		8:[0,1],
	}
	sensor_directions = {
		2:"front",
		3:"front right",
		4:"right",
		5:"reverse right",
		6:"reverse",
		7:"reverse left",
		0:"left",
		1:"front left",
	}
	movement_directions = {
		0:"stop",
		1:"front",
		2:"front right",
		3:"right",
		4:"reverse right",
		5:"reverse",
		6:"reverse left",
		7:"left",
		8:"front left",
	}
	

	def __init__(self, cliff_threshold = 0.03):
		self.cliff_depth = None
		self.point_cloud = None
		self.calibrated = False
		self.calibrated_cliff_depth = None
		self.cal_counter = 5
		self.movement_mode = 0
		self.cliff_threshold = cliff_threshold
		self.obstacle_threshold = 0.3

	def check_obstacle_safety(self):
		# checks if it is safe to move in the direction the robot is currently moving
		
		mode = self.movement_mode

		if(mode == 0):
			return True
		
		# Checks for all sensors in the direction of movement
		if self.point_cloud is None:
			return True
		checklist = self.sensor_check_list[mode]
		for sensor in checklist:
			dist_to_obstacle = self.get_distance_from_point(self.point_cloud[sensor])
			if(dist_to_obstacle < self.obstacle_threshold):
				rospy.loginfo("Obstacle detected by sensor: {}".format(self.sensor_directions[sensor]))
				return False
		return True			
			
	def check_cliff_safety(self):
		if self.cliff_depth is None:
			return True
		if self.movement_mode in [1,2,8]:
			if not self.within_threshold(self.cliff_depth, self.calibrated_cliff_depth, self.cliff_threshold):
				rospy.loginfo("Cliff Detected.")
				return False
		return True

	def cliff_callback(self,data):
		global pub
		#Calibration for initial sensor height
		if self.cal_counter > 0:
			rospy.loginfo("Calibrating depth...")
			if(self.calibrated_cliff_depth is None):
				self.calibrated_cliff_depth = data.data
			else:
				self.calibrated_cliff_depth = 0.8*self.calibrated_cliff_depth + 0.2*data.data
			self.cal_counter -= 1
			if(self.cal_counter == 0):
				self.calibrated = True
				rospy.loginfo("Calibration Complete. Distance to floor: {}m".format(self.calibrated_cliff_depth))
			return

		self.cliff_depth = data.data

		if not self.check_cliff_safety():
			rospy.loginfo("Stopping Movement")
			pub.publish(twist_zero_msg)
			self.movement_mode = 0
		

	def sonar_callback(self,data):
		global pub
		self.point_cloud = data.points
		if not self.check_obstacle_safety():
			rospy.loginfo("Stopping Movement")
			pub.publish(twist_zero_msg)
			self.movement_mode = 0

	def movement_callback(self, data):
		global pub
		if(not self.calibrated):
			rospy.loginfo("Can't Move, Calibrating")
			return
		self.movement_mode = self.determine_movement_mode(data)
		msg = "depth: {}\npoints: {}\nMovement Mode: {}".format(self.cliff_depth, self.point_cloud, self.movement_directions[self.movement_mode])
		rospy.loginfo(msg)
		if self.check_obstacle_safety() and self.check_cliff_safety():
			rospy.loginfo("published: {}".format(data))
			pub.publish(data)
		else:
			rospy.loginfo("Can't move, obstacle or cliff detected")

	@staticmethod
	def within_threshold(test_val, ref_val, threshold):
		if test_val > ref_val + threshold or test_val < ref_val - threshold:
			return False
		return True
	
	@staticmethod
	def get_distance_from_point(point):
		return (point.x**2+point.y**2)**0.5
	
	@staticmethod
	def determine_movement_mode(twist_msg):
		x = twist_msg.linear.x
		z = twist_msg.angular.z
		if x>0:
			if z<0 :
				return 2
			if z>0:
				return 8
			return 1
		if x<0:
			if z<0:
				return 6
			if z>0:
				return 4
			return 5
		if x==0.0:
			if z<0:
				return 3
			if z>0:
				return 7
			return 0


def cliff_listener():
	global pub
	rospy.init_node("cliff_listener", anonymous=True)
	pub = rospy.Publisher("ros0xrobot/cmd_vel", Twist, queue_size=10)
	serv = Server()
	rospy.Subscriber("arduino_dist", Float32,serv.cliff_callback)
	rospy.Subscriber("ros0xrobot/sonar", PointCloud, serv.sonar_callback)
	rospy.Subscriber("ros0xrobot/mqtt_vel", Twist, serv.movement_callback)
	rospy.spin()

if __name__=='__main__':
	cliff_listener()
