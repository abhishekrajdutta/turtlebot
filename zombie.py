#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import roslib
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import cmath
from sensor_msgs.msg import LaserScan


class GoForward():
	def __init__(self):
		rospy.init_node('turnInPlace', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.state=0;
		self.omega=0;
		self.zcurr=0;
		self.zinit=0;
		reset=0;
		r = rospy.Rate(10);
		self.move_cmd = Twist()
		

		rospy.Subscriber("/odom",Odometry,self.OdomCallback)
		rospy.Subscriber("/goal_shouter",Float64,self.GoalCallback)
		# rospy.Subscriber('/scan', LaserScan, self.scanCallback)
		sub = rospy.Subscriber('/scan', LaserScan, call_back)
		
		while not rospy.is_shutdown():
			self.move_cmd.angular.z = self.omega
			self.cmd_vel.publish(self.move_cmd)
			r.sleep()

		rospy.spin()		
		


	def OdomCallback(self,data):
		K=1;
		wmax=1;
		zdes= cmath.rect(1,self.state)
		self.zcurr=(data.pose.pose.orientation.w + data.pose.pose.orientation.z*1j )**2
		rospy.loginfo(zdes)
		zerr=zdes/self.zcurr
		theta=cmath.phase(zerr)
		rospy.loginfo(theta)
		w=K*theta
		if w>wmax:
			w=wmax
		self.omega=w

	
		# num = len(data.ranges)/5
		# temp=np.array(data.ranges)
		# where_is_nans=np.isnan(temp);
		# temp[where_is_nans]=7;
		# 
		# 


	def GoalCallback(self,data):
		self.state=data.data+cmath.phase(self.zcurr)

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
	    	


# def call_back(scanmsg):
#     '''Passes laser scan message to for_callback function of sub_obj.

#     Parameter scanmsg is laserscan message.'''
#     sub_obj.ScanCallback(scanmsg)


# def ScanCallback(self,data):
# 		rospy.loginfo("i work")

if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("turnInPlace node terminated.")