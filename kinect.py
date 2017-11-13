#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import matplotlib.pyplot as plt
import numpy as np
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

		while not rospy.is_shutdown():
			cv2.imshow('image',self.cv_image)
			cv2.imshow('image2',self.dp_image)
			cv2.imshow('image3',self.norm_image)
			cv2.imshow('image4',self.seg_image)
			cv2.waitKey(10);
		
		rospy.spin();

	def colorCallback(self,data):
		self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		boundaries = [([25, 25, 25], [35, 35, 35])]

		for (lower, upper) in boundaries:
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
	 
		
		mask = cv2.inRange(self.cv_image, lower, upper)
		self.seg_image=cv2.bitwise_and(self.cv_image, self.cv_image, mask = mask)
	
		

	def depthCallback(self,data):
		self.dp_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
		dst = np.zeros(shape=(5,2))
		self.norm_image = cv2.normalize(self.dp_image,dst, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
		

if __name__ == '__main__':
	try:
		read_kinect()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")
