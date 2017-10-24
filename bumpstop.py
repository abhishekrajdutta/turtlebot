#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import roslib
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent


class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.state=0;
		reset=0;
		r = rospy.Rate(10);
		self.move_cmd = Twist()
		

		rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)



		
		while not rospy.is_shutdown():
			if self.state==0 and reset==0:
				self.move_cmd.linear.x = 0.1
				self.move_cmd.angular.z = 0
				self.cmd_vel.publish(self.move_cmd)
				r.sleep()

			elif self.state==1 and reset==0:
				self.move_cmd.linear.x = 0
				self.move_cmd.angular.z = 0
				self.cmd_vel.publish(self.move_cmd)
				reset=1

			elif self.state==0 and reset==1:
				rospy.sleep(2)
				reset=0

		rospy.spin()		
		

	def BumperEventCallback(self,data):
	    if ( data.state == BumperEvent.RELEASED ) :
			self.state = 0
	    else:
	    	self.state = 1
	    	

	def WheelDropEventCallback(self,data):
	    if ( data.state == WheelDropEvent.RAISED ) :
			self.state = 0
	    else:
	    	self.state = 1

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
	    	



if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")