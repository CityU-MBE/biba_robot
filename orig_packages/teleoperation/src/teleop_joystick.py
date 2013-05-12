#!/usr/bin/env python
import roslib; roslib.load_manifest('teleoperation')
import rospy
import math
import getopt
import sys
import pygame
from pygame.locals import *
import pygame.sprite 
from geometry_msgs.msg import Twist
from joy.msg import Joy
from std_msgs.msg import Int16, Bool

class Transmitter:
    def __init__(self):
        self.doNotSend = 0
        self.robot_name =  rospy.get_param("~robot_name", "robot")
        self.cmdOutput =  rospy.get_param("~cmdOutput", "/pioneer/cmd_vel")
        self.linear_gain = rospy.get_param(self.robot_name+"/max_linear_vel", 0.05)
        self.angular_gain = rospy.get_param(self.robot_name+"/max_angular_vel", 0.1)
        rospy.loginfo("Max speed from parameter: %s" % self.robot_name+"/max_linear_vel")
    
    def joyCallBack(self,data):
        max_speed = self.linear_gain; # m/second
        max_turn = self.angular_gain; # rad/second
        self.doNotSend = 1
        tw = Twist()

        pub = rospy.Publisher(self.cmdOutput, Twist)
        
        # Emergency stop with the joystick, if this button is press
        # command with 0 will be send if the stick is not touch
        if data.buttons[7] == True:
            
            tw.linear.x = data.axes[1]  * self.linear_gain
            tw.angular.z = data.axes[0]  * self.angular_gain
                
            if tw.linear.x > max_speed:
                tw.linear.x = max_speed
            if tw.linear.x < -max_speed:
                tw.linear.x = -max_speed
            if tw.angular.z > max_turn:
                tw.angular.z = max_turn
            if tw.angular.z < -max_turn:
                tw.angular.z = -max_turn
               
            
            pub.publish(tw)

        
if __name__ == '__main__':
    try:
        rospy.init_node('teleop_py_joy')

        trans = Transmitter()
        rospy.loginfo("Using maximum velocities: linear %.2f angular %.2f" \
                % (trans.linear_gain,trans.angular_gain))
        
        
        rospy.Subscriber('/biba/joy', Joy, trans.joyCallBack)
        rospy.spin()
    except rospy.ROSInterruptException: pass

