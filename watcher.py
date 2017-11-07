#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import cmath

zcurr=1+0j
transmit=1
state=0
omega=0

class Scan_msg:

    
    def __init__(self):
	'''Initializes an object of this class.

	The constructor creates a publisher, a twist message.
	3 integer variables are created to keep track of where obstacles exist.
	3 dictionaries are to keep track of the movement and log messages.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
	
	self.sect = np.zeros(5);
	
	self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}
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
	sect1=np.min(temp[num*2:num*3]);
	if sect1>0.7:
		index=0;
	else:
		sect=np.array([np.mean(temp[num:num*2]),np.mean(temp[num*3:num*4]),np.mean(temp[0:num]),np.mean(temp[num*4:num*5])])
		index=1+np.argmax(sect);
	
	self.movement(index);
	rospy.loginfo(index)
	# elif index==1:
	# 	rospy.loginfo("right")
	# elif index==2:
	# 	rospy.loginfo("left")
	# elif index==3:
	# 	rospy.loginfo("hard left")
	# elif index==4:
	# 	rospy.loginfo("hard right")
	# elif index==5:
	# 	rospy.loginfo("stop!!")	

	


    def movement(self,index):
    	global transmit
    	if index==0 and transmit ==1:
			forward()
	else:
			angle=[75,-75,90,-90]
			if transmit==1:
				goal(3.14/180*angle[index-1])
			transmit=0;
			stop()
			turn()
			
			
			
			

	


	

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

def forward():
	pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	msg = Twist()
	msg.linear.x = 0.25
	msg.angular.z = 0
	pub.publish(msg)

def stop():
	pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	msg = Twist()
	msg.linear.x = 0
	msg.angular.z = 0
	pub.publish(msg)

def turn():
	pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	msg = Twist()
	msg.linear.x = 0
	msg.angular.z = omega
	pub.publish(msg)
	# rospy.lognfo("here")

def goal(angle):
	global state
	state=angle+cmath.phase(zcurr)
		
def OdomCallback(data):
	global zcurr,state,omega,transmit
	K=1;
	wmax=1;
	zdes= cmath.rect(1,state)
	zcurr=(data.pose.pose.orientation.w + data.pose.pose.orientation.z*1j )**2
	# rospy.loginfo(zdes)
	zerr=zdes/zcurr
	theta=cmath.phase(zerr)
	if theta<0.01:
		transmit=1;
	w=K*theta
	if w>wmax:
		w=wmax
	omega=w


def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    sub2 =rospy.Subscriber("/odom",Odometry,OdomCallback)
    rospy.spin()

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener()

