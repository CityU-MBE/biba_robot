#!/usr/bin/env python
import roslib; roslib.load_manifest('teleoperation')
import rospy
import getopt
import sys
import math
import pygame
from pygame.locals import *
import pygame.sprite 
from geometry_msgs.msg import Twist
from joy.msg import Joy
from std_msgs.msg import Int16, Bool

class Transmitter:
	def __init__(self,nodename):
		self.doNotSend = 0
		self.nodeName = nodename
		self.linear_gain = rospy.get_param("~max_linear_vel",0.1)
		self.angular_gain = rospy.get_param("~max_angular_vel",0.1)
	
	#~ def joyCallBack(self,data):
		#~ max_speed = 0.500; # m/second
		#~ max_turn = 50.0*math.pi/180.0; # rad/second
		#~ self.doNotSend = 1
		#~ tw = Twist()
		#~ pub = rospy.Publisher('cmd_vel', Twist)
		#~ if data.buttons[7] == False:
			
			#~ tw.linear.x = data.axes[1]  * 0.5
			#~ tw.angular.z = data.axes[0]  * 0.5
				
			#~ if tw.linear.x > max_speed:
				#~ tw.linear.x = max_speed
			#~ if tw.angular.z > max_turn:
				#~ tw.angular.z = max_turn
				
			#~ pub.publish(tw)
		#~ else:
			#~ set_vel_ctrl_on = rospy.Publisher('set_vel_ctrl_on', Bool)
			#~ set_vel_ctrl_on.publish(False)
			
			#~ set_pan_vel = rospy.Publisher('set_pan_vel', Int16)
			#~ if rospy.has_param('/ptu46_175/default_vel_pan'):
				#~ panVel = int(rospy.get_param('/ptu46_175/default_vel_pan'))
				#~ set_pan_vel.publish(panVel)

			#~ set_tilt_vel = rospy.Publisher('set_tilt_vel', Int16)
			#~ if rospy.has_param('/ptu46_175/default_vel_tilt'):
				#~ tiltVel = int(rospy.get_param('/ptu46_175/default_vel_tilt'))
				#~ set_tilt_vel.publish(tiltVel)
			
			#~ set_pan_pos = rospy.Publisher('set_pan_pos', Int16)
			#~ if rospy.has_param('/ptu46_175/max_pos_pan'):
				#~ panPos = int(rospy.get_param('/ptu46_175/max_pos_pan') * data.axes[0] * 0.75)
				#~ set_pan_pos.publish(panPos)

			#~ set_tilt_pos = rospy.Publisher('set_tilt_pos', Int16)
			#~ if rospy.has_param('/ptu46_175/max_pos_tilt'):
				#~ tiltPos = int(rospy.get_param('/ptu46_175/max_pos_tilt') * data.axes[1] * 0.75)
				#~ set_tilt_pos.publish(tiltPos)



	def kbrd(self):
		pygame.init()
		pygame.key.set_repeat(20, 20)
		window = pygame.display.set_mode((800,30))
		pygame.display.set_caption('Control the robot: UP,DOWN,LEFT,RIGHT, a=accelerate, y=deceleration, esc=quit')
		max_speed = self.linear_gain; # m/second
		max_turn = self.angular_gain; # rad/second
		pub = rospy.Publisher(self.nodeName+'/cmd_vel', Twist)
		

		tw = Twist()
		while not rospy.is_shutdown():
			x = 0
			z = 0
			pygame.event.pump()
			key=pygame.key.get_pressed()  
			if key[pygame.K_LEFT]: z=1
			if key[pygame.K_RIGHT]:z=-1
			if key[pygame.K_UP]:   x=1
			if key[pygame.K_DOWN]: x=-1
			if key[pygame.K_a]:
				max_speed *=1.1
				max_turn *= 1.1
			if key[pygame.K_y]:
				max_speed *= 0.9
				max_turn *= 0.9
			if key[K_ESCAPE]:
				tw.linear.x = 0.0
				tw.angular.z = 0.0 
				pub.publish(tw)
				break

			tw.linear.x = x * max_speed
			tw.angular.z = z *  max_turn

			if self.doNotSend == 1:
				break
				
			pub.publish(tw)
			#rospy.spinOnce()
			rospy.sleep(0.05)
			
		pygame.quit()
	
def printhelp():
	print """
ROS keyboard teleoperation
Options:
    -n, --name=       : LOS robot node
    -h, --help        : this message
"""
	sys.exit(0)

	
if __name__ == '__main__':
	try:
		nodename="robot"
		options, others = getopt.getopt(sys.argv[1:],"hn:",\
				['help','name='])
		for o, v in options:
			if o in ("-h","--help"):
				printhelp
			elif o in ("-n","--name"):
				nodename = v
			else:
				pass

		rospy.init_node('teleop')
		trans = Transmitter(nodename)
		rospy.loginfo("Using maximum velocities: linear %.2f angular %.2f" \
				% (trans.linear_gain,trans.angular_gain))
		#rospy.Subscriber('joy', Joy, trans.joyCallBack)
		trans.kbrd()
		#rospy.spin()
	except rospy.ROSInterruptException: pass
