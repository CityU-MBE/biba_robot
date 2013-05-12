#!/usr/bin/env python
import roslib; roslib.load_manifest('arbiter')
import rospy
import getopt
import sys
import math
from geometry_msgs.msg import Twist
from arbiter.srv import *

class CallablePublisher:
    def __init__(self,mother,id):
        self.mother = mother
        self.id = id
        self.watch=1
        self.count=0
        self.old_vel_cmd = Twist ()
        self.old_vel_cmd.linear.x=0
        self.old_vel_cmd.linear.y=0
        self.old_vel_cmd.angular.z=0
        self.max_acc_x = rospy.get_param("~max_acc_x",0.1)
        self.max_acc_z = rospy.get_param("~max_acc_z",0.1)
        self.seconds = rospy.get_time()

    def __call__(self,x):
        #rospy.loginfo("Received %s from %d" % (str(x),self.id))
        
        # If we receive commands from manual input, force to use this
        # over other inputs.
        if (self.id == 1):
            self.mother.activeService = self.id
        
        if (self.mother.activeService == self.id):
            #rospy.loginfo("Publishing it")
            time = rospy.get_time() - self.seconds
            #rospy.loginfo(time)
            if time > 1.0:
                self.seconds = rospy.get_time()
                rospy.loginfo("Back to manual control")
            else:
                #Acceleration ramp to avoid jerk, don't work if the seb=nd command is 0, because, 0 can be an emergency stop
                if (x.linear.x != 0):
                    if (x.linear.x - self.old_vel_cmd.linear.x > self.max_acc_x*time):
                        x.linear.x = self.old_vel_cmd.linear.x + self.max_acc_x*time
                    if (x.linear.x - self.old_vel_cmd.linear.x < -self.max_acc_x*time):
                        x.linear.x = self.old_vel_cmd.linear.x - self.max_acc_x*time
                if (x.angular.z != 0):
                    if (x.angular.z - self.old_vel_cmd.angular.z > self.max_acc_z*time):
                        x.angular.z = self.old_vel_cmd.angular.z + self.max_acc_z*time
                    if (x.angular.z - self.old_vel_cmd.angular.z < -self.max_acc_z*time):
                        x.angular.z = self.old_vel_cmd.angular.z - self.max_acc_z*time

                #if (x.linear.x > self.mother.linear_gain):
                #    x.linear.x = self.mother.linear_gain

                #if (x.angular.z > self.mother.angular_gain):
                #    x.angular.z = self.mother.angular_gain

                self.mother.pub.publish(x)
                self.old_vel_cmd = x
                self.seconds = rospy.get_time()
            self.watch=1

class Arbiter:
    def __init__(self):
        self.activeService = rospy.get_param("~select",0)
        self.input1 = rospy.get_param("~input_manual","/robot/cmd_vel")
        self.input2 = rospy.get_param("~input_auto","/robot/cmd_vel")
        self.output = rospy.get_param("~output","/robot/cmd_vel")

        self.robot_name = rospy.get_param("~robot_name", "/robot")
        self.linear_gain = rospy.get_param(self.robot_name+"/max_linear_vel",0.1)
        self.angular_gain = rospy.get_param(self.robot_name+"/max_angular_vel",0.1)
        self.pub = rospy.Publisher(self.output, Twist)

        #rospy.loginfo("Max linear vel: %f, from %s" %\
                #(self.linear_gain, self.robot_name+"/max_linear_vel"))
        #rospy.loginfo("Max angular vel: %f, from %s" %\
                #(self.angular_gain, self.robot_name+"/max_angular_vel"))

        self.input_cbo_1 = CallablePublisher(self,1)
        self.input_cbo_2 = CallablePublisher(self,2)

        self.listen1 = rospy.Subscriber(self.input1, Twist, self.input_cbo_1)
        self.listen2 = rospy.Subscriber(self.input2, Twist, self.input_cbo_2)
        self.service = rospy.Service('~select_input', ArbiterSelectInput, self)

    def __call__(self,req):
        #rospy.loginfo("Input request %d" % req.input)
        if req.input == 1:
            self.activeService = 1
            rospy.loginfo("Force manual control")
            return ArbiterSelectInputResponse(0)
        if req.input == 2:
            self.activeService = 2
            rospy.loginfo("Selected auto control, will have priority if commands applied")
            return ArbiterSelectInputResponse(0)
        self.activeService = 0
        #rospy.loginfo("No input selected")
        return ArbiterSelectInputResponse(-1)
    
def printhelp():
    print """
ROS input arbiter
"""
    sys.exit(0)

    
if __name__ == '__main__':
    try:

        rospy.init_node('arbiter')
        
        arbiter = Arbiter()
        #rospy.loginfo("Input 1:%s 2:%s Output %s"\
                #% (arbiter.input1,arbiter.input2,arbiter.output))
        
        freq = rospy.get_param("~freq",5)
        
        r = rospy.Rate(freq)
        count = 0
        count1 = 0
        limit_count_sec = rospy.get_param("~limit_count_sec",5)

        limit_count=limit_count_sec * freq
        
        while not rospy.is_shutdown():

            if arbiter.input_cbo_1.watch == 1:
                #rospy.loginfo("active 1")
                arbiter.input_cbo_1.watch = 0
                count = 0
                count1 = 0

            elif arbiter.input_cbo_1.watch == 0:
                #rospy.loginfo("####inactive")
                count1 = count1 + 1            

            if count1>10:

                # This will force arbiter to get back to original input selection
                arbiter.activeService = rospy.get_param("~select",0)

                if arbiter.input_cbo_2.watch == 1:
                    #rospy.loginfo("active 2")
                    arbiter.input_cbo_2.watch = 0
                    count = 0

                elif arbiter.input_cbo_1.watch == 0 and arbiter.input_cbo_2.watch == 0:
                    #rospy.loginfo("####inactive")
                    count = count + 1

            if count > limit_count:
                null_cmd = Twist()                
                null_cmd.linear.x=0
                null_cmd.linear.y=0
                null_cmd.angular.z=0
                count = 0
                #rospy.loginfo("########limit_count")
                arbiter.pub.publish(null_cmd)


            
            
            r.sleep()

    except rospy.ROSInterruptException: pass
