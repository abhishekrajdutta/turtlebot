#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class Scan_msg:

    
    def __init__(self):
	'''Initializes an object of this class.

	The constructor creates a publisher, a twist message.
	3 integer variables are created to keep track of where obstacles exist.
	3 dictionaries are to keep track of the movement and log messages.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
	# rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
	# rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
	
	self.sect = np.zeros(5);
	# self.sec=
	# self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
	# self.fwd = {0:.25,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}
	self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}
	# self.ang=[-0.8,-0.5,0,0.5,0.8]; #+ve - left -ve right
	# self.fwd=[0.1, 0.2, 0.5, 0.2 , 0.1];

	self.ang=[0,-0.5,0.5,0,0];
	self.fwd=[0.25,0.1,0.1,0,0,0];

    def sort(self, laserscan):
	'''Goes through 'ranges' array in laserscan message and determines 
	where obstacles are located. The class variables sect_1, sect_2, 
	and sect_3 are updated as either '0' (no obstacles within 0.7 m)
	or '1' (obstacles within 0.7 m)

	Parameter laserscan is a laserscan message.'''
	index=0
	num = len(laserscan.ranges)/5
	temp=np.array(laserscan.ranges)
	where_is_nans=np.isnan(temp);
	temp[where_is_nans]=7;
	self.sect1=np.min(temp[num*2:num*3]);
	if self.sect1>1:
		index=0;
	elif 0.4<self.sect1<1:
		self.sect2=np.mean(temp[num:num*2]); #left
		self.sect3=np.mean(temp[num*3:num*4]); #right
		if self.sect2>self.sect3:
			index=2;
		else:
			index=1;

	elif self.sect1<0.4:
		index=5;

	rospy.loginfo(self.sect1);

	# self.sect=np.array([1*np.sum(temp[num*2:num*3]),1*np.sum(temp[num:num*2]),1*np.sum(temp[num*3:num*4]),0.5*np.sum(temp[0:num]),0.8*np.sum(temp[num*4:num*5])])
	# self.sect=np.array([np.sum(temp[num*2:num*3]),np.sum(temp[num:num*2])])
	# index=np.argmax(self.sect);

	# rospy.loginfo(index);	
	# if (self.sect[index]/128)<=1:
	# 	index=5

	# rospy.loginfo(self.sect);
	# rospy.loginfo(temp);
	
	if index==0:
		rospy.loginfo("straight")
	elif index==1:
		rospy.loginfo("right")
	elif index==2:
		rospy.loginfo("left")
	elif index==3:
		rospy.loginfo("hard left")
	elif index==4:
		rospy.loginfo("hard right")
	elif index==5:
		rospy.loginfo("stop!!")	



	self.movement(index);


    def movement(self, index):
		'''Uses the information known about the obstacles to move robot.

		Parameters are class variables and are used to assign a value to
		variable sect and then	set the appropriate angular and linear 
		velocities, and log messages.
		These are published and the sect variables are reset.'''
		# sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
		# rospy.loginfo("Sect = " + str(sect))
		msg = Twist()
		msg.angular.z = self.ang[index]
		msg.linear.x = self.fwd[index]
		self.pub.publish(msg)
		# rospy.loginfo(msg)

		# self.reset_sect()		
				



 
    def for_callback(self,laserscan):
	'''Passes laserscan onto function sort which gives the sect 
	variables the proper values.  Then the movement function is run 
	with the class sect variables as parameters.

	Parameter laserscan is received from callback function.'''
	self.sort(laserscan)
	# self.movement(self.sect)
	

def call_back(scanmsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(scanmsg)

# def BumperEventCallback(self,data):
# 	    if ( data.state == BumperEvent.RELEASED ) :
# 			self.state = 0
# 	    else:
# 	    	self.state = 1
	    	
# def WheelDropEventCallback(self,data):
# 	    if ( data.state == WheelDropEvent.RAISED ) :
# 			self.state = 0
# 	    else:
# 	    	self.state = 1

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    rospy.spin()

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener()
