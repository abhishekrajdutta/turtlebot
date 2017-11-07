#!/usr/bin/env python

import rospy

import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import String


def detect():
	pub = rospy.Publisher('/goal_shouter', Float64, queue_size=10, latch=True)
	while (1):
		
		try:
			mode=float(raw_input('Input:'))
			mode=mode*3.14/180;
			pub.publish(mode)
		except ValueError:
			break



def main():
	global count
	count=0
	rospy.init_node('sender_node') 
	detect()
	rospy.spin()

if __name__== '__main__':
    main()

