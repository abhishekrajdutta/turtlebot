#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist

class read_kinect():


	def __init__(self):
		rospy.init_node("read_kinect")
		rospy.Subscriber("/camera/rgb/image_raw",Image,self.colorCallback)
		rospy.Subscriber("/camera/depth/image_raw",Image,self.depthCallback)
		self.bridge = CvBridge()
		self.cv_image=0;
		self.dp_image=0;
		self.norm_image=0;
		self.seg_image=0;
		self.dist_image=0;
		self.norm_depth=0;
		self.ang_vel = 0.000;
		self.lin_vel = 0.000;
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		move_cmd = Twist()
		r = rospy.Rate(10)

		while not rospy.is_shutdown():
			# cv2.imshow('color image',self.cv_image)
			# cv2.imshow('norm image',self.norm_image)
			cv2.imshow('depth image',self.dp_image)
			cv2.imshow('normdepth image',self.norm_depth)
			cv2.imshow('distance image',self.dist_image)
			# cv2.imshow('segmented image',self.seg_image)
			cv2.waitKey(10);
			move_cmd.linear.x = self.lin_vel
			move_cmd.angular.z = self.ang_vel
			if(self.lin_vel > 0 and self.ang_vel > 0):
				print 'FL'

			elif(self.lin_vel < 0 and self.ang_vel > 0):
				print 'FR'
			elif(self.lin_vel > 0 and self.ang_vel > 0):
				print 'BL'
			elif(self.lin_vel < 0 and self.ang_vel < 0):
				print 'BR'
	



			self.cmd_vel.publish(move_cmd)
			r.sleep()
		
		
		rospy.spin();

	def colorCallback(self,data):
		self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		boundaries = [([130, 50, 200], [190, 90, 255])]

		for (lower, upper) in boundaries:
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
	 	
		self.seg_image = cv2.inRange(self.cv_image, lower, upper,self.seg_image)
		
		max_y, max_x = np.shape(self.seg_image)
		

		ind_x = np.where(np.sum(self.seg_image, axis = 0) != 0)
		ind_y = np.where(np.sum(self.seg_image, axis = 1) != 0)

		centroid_x = np.mean(ind_x);
		centroid_y = np.mean(ind_y);

		control=centroid_x - (max_x/2)
		if abs(control)<25:
			control=0
		
		# print centroid_x, centroid_y, self.seg_image.shape

		self.ang_vel = -0.1* (control)
		if ( self.ang_vel >= 0.5 ):
			self.ang_vel = 0.5
			
		elif( self.ang_vel <= -0.5 ):
			self.ang_vel = -0.5
			

		# self.seg_image=cv2.bitwise_and(self.cv_image, self.cv_image, mask = mask)

	def depthCallback(self,data):
		self.dp_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
		dst = np.zeros(shape=(5,2))		
		self.norm_depth= cv2.normalize(self.dp_image,dst, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
		# rospy.loginfo(np.shape(self.seg_image))
		self.dist_image=np.multiply(self.norm_depth,self.seg_image/255.0)
		self.distance=np.mean(self.dist_image)
		# rospy.loginfo(self.distance)

		ind = self.dist_image[np.where(self.dist_image != 0)]

		centroid_depth = np.mean(ind);

		# print centroid_depth, self.seg_image.shape

		self.lin_vel =  (centroid_depth - 0.1)
		if ( self.lin_vel >= 0.5 ):
			self.lin_vel = 0.5
		elif( self.lin_vel <= -0.5 ):
			self.lin_vel = -0.5




		#detect size of depth image
		# temp=np.array(laserscan.ranges)
		# where_is_nans=np.isnan(temp);
		# temp[where_is_nans]=7;
		

if __name__ == '__main__':
	try:
		read_kinect()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")
