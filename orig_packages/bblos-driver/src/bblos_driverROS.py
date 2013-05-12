#!/usr/bin/python

import roslib; roslib.load_manifest('bblos-driver')
import rospy
"""\
Autonomous motion with odometry loopback and scan retrieval
Copyright (C) 2008 BlueBotics SA
"""

import traceback
import sys
import time
import getopt
import math
import subprocess

from Los.AntPlatform import *       # Contains gf* and pt* constants
from Los.Client import Connection
from Los.Types import *             # Contains all serializable types

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

import tf

# desired commands
timeout = 2.0
lastreqtime = -2.
velreq = 0.0
omegareq = 0.0
publaser = None
pubodo = None
getMeasuredSpeed = False
getIntensities = False
testPlatform = False

loshost_default="192.168.5.1"
losport_default=1234
wheelRadius = 0.08;
globalFrameName="/biba_odom"
cmdInput="/cmd_vel"




def control_callback(data):
    global velreq
    global omegareq
    global lastreqtime
    lastreqtime = rospy.rostime.get_time()
    velreq = data.linear.x
    omegareq = data.angular.z
    # rospy.loginfo("Speed req %f and %f" % (velreq,omegareq))

    
def bblos_main(address):

    rospy.loginfo ('bblos_main')
    """Read scans from the platform."""
    global getIntensities
    global getMeasuredSpeed
    global testPlatform
    global timeout
    global lastreqtime
    global velreq
    global omegareq
    global pubodo
    global publaser

    # pre-initialise the constant part of the messages
    odomsg = Odometry()
    odomsg.header.frame_id = globalFrameName
    odomsg.child_frame_id = "biba_base";
    odomsg.pose.pose.position.z = 0
    for i in range(0,36):
        odomsg.pose.covariance[i] = 0
    for i in range(0,36):
        odomsg.twist.covariance[i] = 0
    odomsg.twist.twist.linear.y = 0
    odomsg.twist.twist.linear.z = 0
    odomsg.twist.twist.angular.x = 0
    odomsg.twist.twist.angular.y = 0
    
    laser = PointCloud()
    laser.header.frame_id = "laser"
    laser.channels.append(ChannelFloat32())
    laser.points = []
    if getIntensities:
        laser.channels[0].name = "intensities"
        laser.channels[0].values = []


    # Use a Rate object to try to have a constant sampling time (which LOS
    # cannnot provide anyway)
    rate = rospy.Rate(10)
    proxy = Connection(address)
    proxy.login("User", "none")
    while not rospy.is_shutdown():
        # First acquire all the data
        tstamp = rospy.rostime.get_rostime()
        try:
            proxy.Watchdog.reset(3.0)
            # We can de-activate intensities if requested
            if getIntensities:
                (tlaser, pose, maxAge, indices, coordinates, intensities) = proxy.Scan.get(Int32(gfCoordinates | gfSync | gfIntensities))
            else:
                (tlaser, pose, maxAge, indices, coordinates) = proxy.Scan.get(Int32(gfCoordinates | gfSync ))
            #rospy.logdebug("info")
            #rospy.logdebug(tlaser)
            #rospy.logdebug(pose)
            #rospy.logdebug(maxAge)
            #rospy.logdebug(indices)
            #rospy.logdebug("coordinates ") 
            #rospy.logdebug(coordinates)
            #if getIntensities:
            #    rospy.logdebug(intensities)
            
            (todo, lospose) = proxy.Odometry.getPose()

            
            if getMeasuredSpeed:
                # Activate this part if you really need the robot speed
                speeds = proxy.Motion.getSpeed()
            else:
                speeds = [0, velreq, omegareq]
            # Implement a timeout if the last requested control message is too
            # old
            if (rospy.rostime.get_time() - lastreqtime) > timeout:
                velreq = 0.0
                omerareq = 0.0
            # And finally send the latest command
            if testPlatform:
                proxy.Motion.setSpeed(0.1*math.sin(rospy.rostime.get_time()), 0.1*math.cos(rospy.rostime.get_time()))
            else:
                proxy.Motion.setSpeed(velreq,omegareq)
            # rospy.loginfo("Speed sent %f and %f" % (velreq,omegareq))
        except SyntaxError:
            traceback.print_exc()
            print "Exception while accessing los proxy"
            break
        except:
            print "Interrupted"
            break

        # Now convert the LOS type to ROS messages
        n = len(coordinates)/2

        laser.header.stamp = tstamp
        if not (n == len(laser.points)):
            laser.points = [Point32()]*n
            if getIntensities:
                laser.channels[0].values = [0]*n
        for i in range(0,n):
            laser.points[i] = Point32(coordinates[2*i+0], coordinates[2*i+1], 0)
            #laser.points[i].x = Point32(coordinates[2*i+0], coordinates[2*i+1], 0)
            #laser.points[i].y = Point32(coordinates[2*i+1])
            #laser.points[i].z = 0
            #rospy.logdebug("points: %f, %f", laser.points[i].x, laser.points[i].y)
            if getIntensities:
                laser.channels[0].values[i] = intensities[i]
        
        publaser.publish(laser)

        # Directly send requested values
        odomsg.header.stamp = tstamp
        odomsg.twist.twist.linear.x = speeds[1]
        odomsg.twist.twist.angular.z = speeds[2]
        odomsg.pose.pose.position.x = lospose[0];
        odomsg.pose.pose.position.y = lospose[1];
        odomsg.pose.pose.position.z = wheelRadius;
        q = tf.transformations.quaternion_from_euler(0,0,lospose[2])
        odomsg.pose.pose.orientation.x = q[0]
        odomsg.pose.pose.orientation.y = q[1]
        odomsg.pose.pose.orientation.z = q[2]
        odomsg.pose.pose.orientation.w = q[3]
        # Assuming the covariance is Cov[x y 0 . . theta]
        odomsg.pose.covariance[6*0+0] = lospose[3]; # x * x
        odomsg.pose.covariance[6*0+1] = lospose[6]; # x * y
        odomsg.pose.covariance[6*1+0] = lospose[6]; # y * x
        odomsg.pose.covariance[6*1+1] = lospose[4]; # y * y
        odomsg.pose.covariance[6*0+5] = lospose[7]; # x * theta
        odomsg.pose.covariance[6*5+0] = lospose[7]; # theta * x
        odomsg.pose.covariance[6*1+5] = lospose[8]; # y * theta
        odomsg.pose.covariance[6*5+1] = lospose[8]; # theta * y
        odomsg.pose.covariance[6*5+5] = lospose[5]; # theta * theta
        pubodo.publish(odomsg)

        # Publish transformation frame
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (odomsg.pose.pose.position.x, 
                odomsg.pose.pose.position.y, 
                odomsg.pose.pose.position.z),
            (odomsg.pose.pose.orientation.x, 
                odomsg.pose.pose.orientation.y, 
                odomsg.pose.pose.orientation.z,
                odomsg.pose.pose.orientation.w),
            rospy.Time.now(),
            odomsg.child_frame_id,
            globalFrameName
            )

        #rospy.loginfo("published laser and odo")
        rate.sleep()


def main():
    global loshost
    global losport
    global getIntensities
    global getMeasuredSpeed
    global testPlatform
    global wheelRadius
    global globalFrameName
    global cmdInput

    rospy.init_node('biba')
    
    # Get parameters
    print ("~loshost") 
    loshost = rospy.get_param("~loshost",default=loshost_default)
    losport = rospy.get_param("~losport",default=losport_default)
    getIntensities = rospy.get_param("~getIntensities", default=getIntensities)
    getMeasuredSpeed = rospy.get_param("~getMeasuredSpeed", default=getMeasuredSpeed)
    testPlatform = rospy.get_param("~testPlatform", default=testPlatform)
    wheelRadius = rospy.get_param("~wheelRadius", default=wheelRadius)
    globalFrameName= rospy.get_param("~globalFrameName", default=globalFrameName)
    cmdInput= rospy.get_param("~cmdInput", default=cmdInput)

    rospy.loginfo ('main')
    """Launch localization and scan reader threads, and move along a node list."""
    global pubodo
    global publaser
    address = (loshost, losport)     # Platform address
    rospy.loginfo ('host: %s' % loshost)
    rospy.Subscriber(cmdInput, Twist , control_callback)
    pubodo = rospy.Publisher('~odom', Odometry)
    publaser = rospy.Publisher('~laser', PointCloud)
    
    if rospy.has_param("~max_linear_vel"):
        rospy.set_param("~max_linear_vel",0.5)
    if rospy.has_param("~max_angular_vel"):
        rospy.set_param("~max_angular_vel",0.5)

    try:
        bblos_main(address)
    except KeyboardInterrupt:
        print "Interrupted"


    
if __name__ == "__main__":
    sys.exit(main())
