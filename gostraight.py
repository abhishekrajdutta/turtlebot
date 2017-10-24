#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import roslib
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from sensor_msgs.msg import Imu
import cmath


class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.state=0;
		self.omega=0;
		reset=0;
		r = rospy.Rate(10);
		self.move_cmd = Twist()
		

		rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
		rospy.Subscriber("/mobile_base/sensors/imu_data",Imu,self.ImuCallback)



		
		while not rospy.is_shutdown():
			self.move_cmd.linear.x = 0.5
			self.move_cmd.angular.z = self.omega
			self.cmd_vel.publish(self.move_cmd)
			r.sleep()

		rospy.spin()		
		


	def ImuCallback(self,data):
		K=1;
		wmax=1;
		zdes=cmath.rect(1,0);
		zcurr=data.orientation.w + data.orientation.z*1j 
		zerr=zdes/zcurr
		theta=cmath.phase(zerr)
		rospy.loginfo(theta)
		w=K*theta
		if w>wmax:
			w=wmax
		self.omega=w;

	    

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
	    	



if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")