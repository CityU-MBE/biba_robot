#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts

######################################################################
#                                                                    #
#         Modified for robots@home by Quentin Boussardon             #
# Add function to check the errors and the battery level on biba and #
#James, using telnet.                                                #
# Add function to check the connections, using ping.                 #
#                                                                    #
######################################################################


from __future__ import with_statement
import roslib
roslib.load_manifest('diagnostic')

import telnetlib

import rospy
import time
import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string
from std_msgs.msg import *
import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }


# Output entire IPMI data set
def check_ipmi():
    diag_vals = []
    diag_msgs = []
    diag_level = DiagnosticStatus.OK

    try:
        p = subprocess.Popen('sudo ipmitool sdr',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
                        
        if retcode != 0:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = [ 'ipmitool Error' ]
            diag_vals = [ KeyValue(key = 'IPMI Error', value = stderr) ]
            return diag_vals, diag_msgs, diag_level

        lines = stdout.split('\n')
        if len(lines) < 2:
            diag_vals = [ KeyValue(key = 'ipmitool status', value = 'No output') ]

            diag_msgs = [ 'No ipmitool response' ]
            diag_level = DiagnosticStatus.ERROR

            return diag_vals, diag_msgs, diag_level

        for ln in lines:
            if len(ln) < 2:
                continue

            words = ln.split('|')
            name = words[0].strip()
            ipmi_val = words[1].strip()
            stat_byte = words[2].strip()

            # CPU temps
            if words[0].startswith('CPU') and words[0].strip().endswith('Temp'):
                if words[1].strip().endswith('degrees C'):
                    tmp = ipmi_val.rstrip(' degrees C').lstrip()
                    if unicode(tmp).isnumeric():
                        temperature = float(tmp)
                        diag_vals.append(KeyValue(key = name + ' (C)', value = tmp))

                        cpu_name = name.split()[0]
                        if temperature >= 80 and temperature < 89:
                            diag_level = max(diag_level, DiagnosticStatus.WARN)
                            if diag_msgs.count('CPU Hot') == 0:
                                diag_msgs.append('CPU Warm')

                        if temperature >= 89: # CPU should shut down here
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('CPU Hot')                                
                            # Don't keep CPU Warm in list if CPU is hot
                            if diag_msgs.count('CPU Warm') > 0:
                                idx = diag_msgs.index('CPU Warm')
                                diag_msgs.pop(idx)
                else:
                    diag_vals.append(KeyValue(key = name, value = words[1]))


            # MP, BP, FP temps
            if name == 'MB Temp' or name == 'BP Temp' or name == 'FP Temp':
                if ipmi_val.endswith('degrees C'):
                    tmp = ipmi_val.rstrip(' degrees C').lstrip()
                    diag_vals.append(KeyValue(key = name + ' (C)', value = tmp))
                    # Give temp warning
                    dev_name = name.split()[0]
                    if unicode(tmp).isnumeric():
                        temperature = float(tmp)

                        if temperature >= 60 and temperature < 75:
                            diag_level = max(diag_level, DiagnosticStatus.WARN)
                            diag_msgs.append('%s Warm' % dev_name)

                        if temperature >= 75:
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('%s Hot' % dev_name)
                    else:
                        diag_level = max(diag_level, DiagnosticStatus.ERROR)
                        diag_msgs.append('%s Error' % dev_name)
                else:
                    diag_vals.append(KeyValue(key = name, value = ipmi_val))
        
            # CPU fan speeds
            if (name.startswith('CPU') and name.endswith('Fan')) or name == 'MB Fan':
                if ipmi_val.endswith('RPM'):
                    rpm = ipmi_val.rstrip(' RPM').lstrip()
                    if unicode(rpm).isnumeric():
                        if int(rpm) == 0:
                            diag_level = max(diag_level, DiagnosticStatus.WARN)
                            diag_msgs.append('Fan Warning')
                            
                        diag_vals.append(KeyValue(key = name + ' RPM', value = rpm))
                    else:
                        diag_vals.append(KeyValue(key = name, value = ipmi_val))

            # If CPU is hot we get an alarm from ipmitool, report that too
            # CPU should shut down if we get a hot alarm, so report as error
            if name.startswith('CPU') and name.endswith('hot'):
                if ipmi_val == '0x01':
                    diag_vals.append(KeyValue(key = name, value = 'OK'))
                else:
                    diag_vals.append(KeyValue(key = name, value = 'Hot'))
                    diag_level = max(diag_level, DiagnosticStatus.ERROR)
                    diag_msgs.append('CPU Hot Alarm')

    except Exception, e:
        diag_vals.append(KeyValue(key = 'Exception', value = traceback.format_exc()))
        diag_level = DiagnosticStatus.ERROR
        diag_msgs.append('Exception')

    return diag_vals, diag_msgs, diag_level
        

##\brief Check CPU core temps 
##
## Use 'find /sys -name temp1_input' to find cores
## Read from every core, divide by 1000
def check_core_temps(sys_temp_strings):
    diag_vals = []
    diag_level = 0
    diag_msgs = []
    
    for index, temp_str in enumerate(sys_temp_strings):
        if len(temp_str) < 5:
            continue
        
        cmd = 'cat %s' % temp_str
        p = subprocess.Popen(cmd, stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = [ 'Core Temp Error' ]
            diag_vals = [ KeyValue(key = 'Core Temp Error', value = stderr), 
                          KeyValue(key = 'Output', value = stdout) ]
            return diag_vals, diag_msgs, diag_level
  
        tmp = stdout.strip()
        if unicode(tmp).isnumeric():
            temp = float(tmp) / 1000
            diag_vals.append(KeyValue(key = 'Core %d Temp' % index, value = str(temp)))

            if temp >= 85 and temp < 90:
                diag_level = max(diag_level, DiagnosticStatus.WARN)
                diag_msgs.append('Warm')
            if temp >= 90:
                diag_level = max(diag_level, DiagnosticStatus.ERROR)
                diag_msgs.append('Hot')
        else:
            diag_level = max(diag_level, DiagnosticStatus.ERROR) # Error if not numeric value
            diag_vals.append(KeyValue(key = 'Core %s Temp' % index, value = tmp))

    return diag_vals, diag_msgs, diag_level

## Checks clock speed from reading from CPU info
def check_clock_speed(enforce_speed):
    vals = []
    msgs = []
    lvl = DiagnosticStatus.OK

    try:
        p = subprocess.Popen('cat /proc/cpuinfo | grep MHz', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            lvl = DiagnosticStatus.ERROR
            msgs = [ 'Clock speed error' ]
            vals = [ KeyValue(key = 'Clock speed error', value = stderr), 
                     KeyValue(key = 'Output', value = stdout) ]
            
            return (vals, msgs, lvl)

        for index, ln in enumerate(stdout.split('\n')):
            words = ln.split(':')
            if len(words) < 2:
                continue

            speed = words[1].strip().split('.')[0] # Conversion to float doesn't work with decimal
            vals.append(KeyValue(key = 'Core %d MHz' % index, value = speed))
            if unicode(speed).isnumeric():
                mhz = float(speed)
                
                if mhz < 2240 and mhz > 2150:
                    lvl = max(lvl, DiagnosticStatus.WARN)
                if mhz <= 2150:
                    lvl = max(lvl, DiagnosticStatus.ERROR)
            else:
                # Automatically give error if speed isn't a number
                lvl = max(lvl, DiagnosticStatus.ERROR)

        if not enforce_speed:
            lvl = DiagnosticStatus.OK

        if lvl == DiagnosticStatus.WARN and enforce_speed:
            msgs = [ 'Core slowing' ]
        elif lvl == DiagnosticStatus.ERROR and enforce_speed:
            msgs = [ 'Core throttled' ]

    except Exception, e:
        rospy.logerr(traceback.format_exc())
        lvl = DiagnosticStatus.ERROR
        msgs.append('Exception')
        vals.append(KeyValue(key = 'Exception', value = traceback.format_exc()))

    return vals, msgs, lvl
                    

# Add msgs output, too
##\brief Uses 'uptime' to see load average
def check_uptime():
    level = DiagnosticStatus.OK
    vals = []
    
    load_dict = { 0: 'OK', 1: 'High Load', 2: 'Very High Load' }

    try:
        p = subprocess.Popen('uptime', stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            vals.append(KeyValue(key = 'uptime Failed', value = stderr))
            return DiagnosticStatus.ERROR, vals

        upvals = stdout.split()
        load1 = upvals[-3].rstrip(',')
        load5 = upvals[-2].rstrip(',')
        load15 = upvals[-1]
        num_users = upvals[-7]

        # Give error if we go over load limit 
        if float(load1) > 25 or float(load5) > 18:
            level = DiagnosticStatus.WARN

        vals.append(KeyValue(key = 'Load Average Status', value = load_dict[level]))
        vals.append(KeyValue(key = '1 min Load Average', value = load1))
        vals.append(KeyValue(key = '5 min Load Average', value = load5))
        vals.append(KeyValue(key = '15 min Load Average', value = load15))
        vals.append(KeyValue(key = 'Number of Users', value = num_users))

    except Exception, e:
        rospy.logerr(traceback.format_exc())
        level = DiagnosticStatus.ERROR
        vals.append(KeyValue(key = 'Load Average Status', value = traceback.format_exc()))
        
    return level, vals

# Add msgs output
##\brief Uses 'free -m' to check free memory
def check_memory():
    values = []
    level = DiagnosticStatus.OK
    msg = ''

    mem_dict = { 0: 'OK', 1: 'Low Memory', 2: 'Very Low Memory' }

    try:
        p = subprocess.Popen('free -m',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
               
        if retcode != 0:
            values.append(KeyValue(key = "\"free -m\" Call Error", value = str(retcode)))
            return DiagnosticStatus.ERROR, values
 
        rows = stdout.split('\n')
        data = rows[1].split()
        total_mem = data[1]
        used_mem = data[2]
        free_mem = data[3]

        level = DiagnosticStatus.OK
        if float(free_mem) < 25:
            level = DiagnosticStatus.WARN
        if float(free_mem) < 1:
            level = DiagnosticStatus.ERROR

        values.append(KeyValue(key = 'Memory Status', value = mem_dict[level]))
        values.append(KeyValue(key = 'Total Memory', value = total_mem))
        values.append(KeyValue(key = 'Used Memory', value = used_mem))
        values.append(KeyValue(key = 'Free Memory', value = free_mem))

        msg = mem_dict[level]
    except Exception, e:
        rospy.logerr(traceback.format_exc())
        msg = 'Memory Usage Check Error'
        values.append(KeyValue(key = msg, value = str(e)))
        level = DiagnosticStatus.ERROR
    
    return level, values

##\brief Use mpstat to find CPU usage
##
##
usage_old = 0
has_warned_mpstat = False
def check_mpstat():
    vals = []
    mp_level = DiagnosticStatus.OK
    
    load_dict = { 0: 'OK', 1: 'High Load', 2: 'Very High Load' }

    try:
        p = subprocess.Popen('mpstat -P ALL 1 1',
	
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            global has_warned_mpstat
            if not has_warned_mpstat:
                rospy.logerr("mpstat failed to run for cpu_monitor. Return code %d.", retcode)
                has_warned_mpstat = True

            mp_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = '\"mpstat\" Call Error', value = str(retcode)))
            return mp_level, vals

        num_cores = 0
        cores_loaded = 0
        for index, row in enumerate(stdout.split('\n')):
            if index < 3:
                continue
            
            lst = row.split()



            if len(lst) < 10:
                continue
		


            ## Ignore 'Average: ...' data
            if lst[0].startswith('Average'):
	    	continue

            cpu_name = lst[2]
            if cpu_name.strip() == 'all':
		cpu_name = 'ALL'
            idle = lst[11]
            user = lst[3]
            nice = lst[4]
            system = lst[5]
            
            core_level = 0
            usage = float(user) + float(nice)
            if usage > 1000: # wrong reading, use old reading instead
                rospy.logwarn('Read cpu usage of %f percent. Reverting to previous reading of %f percent'%(usage, usage_old))
                usage = usage_old
            usage_old = usage

            num_cores += 1
            if usage > 90.0:
                cores_loaded += 1
                core_level = DiagnosticStatus.WARN
            if usage > 100.0:
                core_level = DiagnosticStatus.ERROR

            vals.append(KeyValue(key = 'CPU %s Status' % cpu_name, value = load_dict[core_level]))
            vals.append(KeyValue(key = 'CPU %s User' % cpu_name.strip(), value = user))
            vals.append(KeyValue(key = 'CPU %s Nice' % cpu_name, value = nice))
            vals.append(KeyValue(key = 'CPU %s System' % cpu_name, value = system))
            vals.append(KeyValue(key = 'CPU %s Idle' % cpu_name, value = idle))
        
        # Warn for high load only if we have <= 2 cores that aren't loaded
        if num_cores - cores_loaded <= 2 and num_cores > 2:
            mp_level = DiagnosticStatus.WARN
            
    except Exception, e:
        mp_level = DiagnosticStatus.ERROR
        vals.append(KeyValue(key = 'mpstat Exception', value = str(e)))

    return mp_level, vals

##############################
#Check Battery level

bat_old = 0
has_warned_batstat = False
def check_bat():
    vals = []
    bat_level = DiagnosticStatus.OK
    
    load_dict = { 0: 'OK', 1: 'Low Battery', 2: 'No Battery' }

    try:
        p = subprocess.Popen('acpi',
	
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            global has_warned_batstat
            if not has_warned_batstat:
                rospy.logerr("acpi failed to run for cpu_monitor. Return code %d.", retcode)
                has_warned_batstat = True

            bat_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = '\"acpi\" Call Error', value = str(retcode)))
            return bat_level, vals
	
        for index, bat in enumerate(stdout.strip().split('\n')):

	    lst = bat.split()

	    bat_name = lst[1]
            
            state = lst[2]
            level = lst[3]
            
	    if level == '100%':
	    	levelc = level[0:3]
	    else:
		levelc = level[0:2]

	                
            usage = float(levelc)
            if usage < 10.0:
               bat_level = DiagnosticStatus.WARN
            
            vals.append(KeyValue(key = 'BAT %s Status' % bat_name, value = load_dict[bat_level]))
            vals.append(KeyValue(key = 'BAT %s State' % bat_name, value = state))
            vals.append(KeyValue(key = 'BAT %s Level' % bat_name, value = levelc))

    except Exception, e:
		bat_level = DiagnosticStatus.ERROR
	        vals.append(KeyValue(key = 'acpi Exception', value = str(e)))
    PC_Voltage_pub.publish(usage)
    return bat_level, vals

##############################
#Check Robot

robot_old = 0
has_warned_robotstat = False
def check_robot():
    vals = []
    robot_level = DiagnosticStatus.OK
    ip_robot=str(rospy.get_param('~ip_robot'))
    load_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }
    time.sleep(1)
    p = subprocess.Popen("ping -c 1 -W 1 %s" %(ip_robot),
            	         stdout = subprocess.PIPE,
                         stderr = subprocess.PIPE, shell = True)
    stdout, stderr = p.communicate()
    retcode = p.returncode
    voltage = 0.0

    if retcode == 0:

        tn = telnetlib.Telnet(ip_robot)
        
        #time.sleep(1)
        if ip_robot == "10.0.1.10":
            # No login and Password, I don't why, the connection was automatic with biba-laptop.
            #If you use another PC, uncomment those 4 lines 
            #tn.read_until("login:")
            #tn.write("BIBA0\r\n")
            #tn.read_until("Password:")
            #tn.write("none\r\n")
            tn.write("error\r\n")
            tn.read_until("BIBA0@none> ")
            #time.sleep(1)
            diag = tn.read_until("BIBA0@none> ")
            tn.write("volt\r\n")
            volt = tn.read_until("BIBA0@none> ")
            tn.write("exit\r\n")
        elif ip_robot == "192.168.5.1":
            tn.read_until("login:")
            tn.write("PPC\r\n")
            tn.read_until("Password:")
            tn.write("none\r\n")
            #time.sleep(1)
            tn.write("error\r\n")
            tn.read_until("PPC@icPPC> ")
            #time.sleep(1)
            diag = tn.read_until("PPC@icPPC> ")
            tn.write("volt\r\n")
            volt = tn.read_until("PPC@icPPC> ")
            tn.write("exit\r\n")
        diag = diag.split('\n')
        volt = volt.split('\n')

        for i in diag[0:len(diag)-1]:
            warn=0
            print i
            if i == "No problems.\r":
                runflag = 0
                RFlag_pub.publish(runflag)
                if robot_level == DiagnosticStatus.WARN:
                    warn=1

                robot_level = DiagnosticStatus.OK
                vals.append(KeyValue(key = 'Robot Status',value = i))
                if warn == 1:
                    robot_level = DiagnosticStatus.WARN

            else:
                robot_level = DiagnosticStatus.WARN
                vals.append(KeyValue(key = 'Robot Status',value = i))
                has_warned_robotstat = True
                runflag = 1
                RFlag_pub.publish(runflag)

        for i in volt[0:len(volt)-1]:
            warn=0
            if robot_level == DiagnosticStatus.WARN:
                warn=1
            print i
            robot_level = DiagnosticStatus.OK
            vals.append(KeyValue(key = 'Robot Status',value = i))
            if warn == 1:
                robot_level = DiagnosticStatus.WARN
            if i[0:5] == "Volta":
                voltage = float(i[9:len(i)-1])
                print voltage
    else:
        robot_level = DiagnosticStatus.ERROR
        vals.append(KeyValue(key = 'Robot Status',value = "The robot is disconnected"))
        print "The robot is disconnected"
    Robot_Voltage_pub.publish(voltage)
    #rospy.set_param('voltage', voltage)
    return robot_level, vals
###############################################################


###############################################################
#Check Network 
#Test connexions and speed
#ip_list is one of teh parameters in param.yaml

net_old = 0
has_warned_netstat = False

def check_net():
    vals = []
    net_level = DiagnosticStatus.OK
    
    load_dict = { 0: 'Connected', 1: 'Slow Connection', 2: 'Disconnected' }

    ip_list=str(rospy.get_param('~ip_list'))
    ip_list=ip_list[1:len(ip_list)-2].strip().split(', ')

    try:

	for i in ip_list:

		#vals.append(KeyValue(key = "%s" %(i), value = 'ok'))       

		ping = "ping -c 1 -W 1 %s" % (i)
	        p = subprocess.Popen(ping,
				     stdout = subprocess.PIPE,
       	                     stderr = subprocess.PIPE, shell = True)
        	stdout, stderr = p.communicate()
        	retcode = p.returncode

        	if retcode != 0:
        	    
        	    net_level = DiagnosticStatus.ERROR
	            vals.append(KeyValue(key = "%s : Status"%i, value = load_dict[net_level]))
        	    continue
	
		rows = stdout.split('\n')
        	data = rows[1].split()
               	res = float(data[6][5:10])
		if res > 10.00:
			if net_level == DiagnosticStatus.ERROR:
				net_level = DiagnosticStatus.WARN
	        		vals.append(KeyValue(key = "%s : Status"%i, value = load_dict[net_level]))
				net_level = DiagnosticStatus.ERROR				
			else:
				net_level = DiagnosticStatus.WARN
	        		vals.append(KeyValue(key = "%s : Status"%i, value = load_dict[net_level]))
	        	vals.append(KeyValue(key = "%s : Connexion speed"%i, value = "%f"%res))
		else:
			net_level_bkp = net_level
			net_level = DiagnosticStatus.OK
	        	vals.append(KeyValue(key = "%s : Status"%i, value = load_dict[net_level]))
	        	vals.append(KeyValue(key = "%s : Connexion speed"%i, value = "%f"%res))
			net_level = net_level_bkp


    except Exception, e:
 	net_level = DiagnosticStatus.ERROR
       	vals.append(KeyValue(key = 'ping Exception', value = str(e)))

    return net_level, vals
##############################
#Check Device
#Test number of connected devices
#dev_list is one of the parameters in param.yaml

dev_old = 0
has_warned_devstat = False

def check_dev():
    vals = []
    dev_level = DiagnosticStatus.OK

    load_dict = { 0: 'OK', 1: 'Too many connected', 2: 'None of connected' }
    dev_check=str(rospy.get_param('~dev_check'))
    dev_list=str(rospy.get_param('~dev_list',True))
    dev_list=dev_list[1:len(dev_list)-2].strip().split(', ')
    if (dev_check == True):
        try:

            for i in dev_list:

                dev=(i[0:len(i)-2]+'*')

                p = subprocess.Popen("ls %s"%dev,
                                     stdout = subprocess.PIPE,
                                     stderr = subprocess.PIPE, shell = True)
                stdout, stderr = p.communicate()
                retcode = p.returncode

                if retcode != 0:
                    dev_level = DiagnosticStatus.ERROR
                    vals.append(KeyValue(key = "%s"%i, value = load_dict[dev_level]))	
                    continue

    #Test number of devices, we should have only one connected, to avoid conflicts

                nb_dev = len(stdout.strip().split('\n'))

                if nb_dev == 0:
                    dev_level = DiagnosticStatus.ERROR
                    vals.append(KeyValue(key = "%s"%i, value = load_dict[dev_level]))	

                elif nb_dev == 1:

    #Test name of device, to assure it's connected to the good port
                    p = subprocess.Popen("ls %s"%i,
                                         stdout = subprocess.PIPE,
                                         stderr = subprocess.PIPE, shell = True)
                    stdout, stderr = p.communicate()
                    retcode = p.returncode

                    if retcode != 0:
                        dev_level = DiagnosticStatus.ERROR
                        vals.append(KeyValue(key = "%s"%i, value = 'wrong adress'))	
                        continue


                    else:
                        dev_level_old = dev_level
                        dev_level = DiagnosticStatus.OK
                        vals.append(KeyValue(key = "%s"%i, value = load_dict[dev_level]))
                        dev_level = dev_level_old

                else:
                    if dev_level == DiagnosticStatus.OK:
                        dev_level = DiagnosticStatus.WARN
                        vals.append(KeyValue(key = "%s"%i, value = load_dict[dev_level]+" : %d detected"%nb_dev))				
                    else:
                        dev_level = DiagnosticStatus.WARN
                        vals.append(KeyValue(key = "%s"%i, value = load_dict[dev_level]+" : %d detected"%nb_dev))
                        dev_level = DiagnosticStatus.ERROR

        except Exception, e:
            dev_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = 'ls Exception', value = str(e)))

    return dev_level, vals


## Returns names for core temperature files
## Returns list of names, each name can be read like file
def get_core_temp_names():
    temp_vals = []
    try:
        p = subprocess.Popen('find /sys/devices -name temp1_input', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            rospy.logerr('Error find core temp locations: %s' % stderr)
            return []
        
        for ln in stdout.split('\n'):
            temp_vals.append(ln.strip())
        
        return temp_vals
    except:
        rospy.logerr('Exception finding temp vals: %s' % traceback.format_exc())
        return []

def update_status_stale(stat, last_update_time):
    time_since_update = rospy.get_time() - last_update_time

    stale_status = 'OK'
    if time_since_update > 20 and time_since_update <= 35:
        stale_status = 'Lagging'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.WARN)
    if time_since_update > 35:
        stale_status = 'Stale'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.ERROR)


    stat.values.pop(0)
    stat.values.pop(0)
    stat.values.insert(0, KeyValue(key = 'Update Status', value = stale_status))
    stat.values.insert(1, KeyValue(key = 'Time Since Update', value = str(time_since_update)))
    

class CPUMonitor():
    def __init__(self, hostname, diag_hostname):
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self._mutex = threading.Lock()

        self._check_core_temps = rospy.get_param('~check_core_temps', True)
        self._check_ipmi = rospy.get_param('~check_ipmi_tool', False)
        self._enforce_speed = rospy.get_param('~enforce_clock_speed', False)
        self._check_nfs = rospy.get_param('~check_nfs', True)
	self._check_bat = rospy.get_param('~check_bat', True)
	self._check_net = rospy.get_param('~check_net', True)	
	self._check_dev = rospy.get_param('~check_dev', True)
	self._check_robot = rospy.get_param('~check_robot', True)

        self._temps_timer = None
        self._usage_timer = None
        self._nfs_timer = None
        self._bat_timer = None
        self._net_timer = None
        self._dev_timer = None
        self._robot_timer = None
        
        # Get temp_input files
        self._temp_vals = get_core_temp_names()

        # CPU stats
        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = '%s CPU Temperature' % diag_hostname
        self._temp_stat.level = 1
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                   KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = '%s CPU Usage' % diag_hostname
        self._usage_stat.level = 1
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._nfs_stat = DiagnosticStatus()
        self._nfs_stat.name = '%s NFS IO' % diag_hostname
        self._nfs_stat.level = 1
        self._nfs_stat.hardware_id = hostname
        self._nfs_stat.message = 'No Data'
        self._nfs_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                  KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._bat_stat = DiagnosticStatus()
        self._bat_stat.name = '%s CPU Battery' % diag_hostname
        self._bat_stat.level = 1
        self._bat_stat.hardware_id = hostname
        self._bat_stat.message = 'No Data'
        self._bat_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                  KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._robot_stat = DiagnosticStatus()
        self._robot_stat.name = 'Robot'
        self._robot_stat.level = 1
        self._robot_stat.hardware_id = hostname
        self._robot_stat.message = 'No Data'
        self._robot_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                  KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._net_stat = DiagnosticStatus()
        self._net_stat.name = '%s Network' % diag_hostname
        self._net_stat.level = 1
        self._net_stat.hardware_id = hostname
        self._net_stat.message = 'No Data'
        self._net_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                  KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._dev_stat = DiagnosticStatus()
        self._dev_stat.name = '%s Device' % diag_hostname
        self._dev_stat.level = 1
        self._dev_stat.hardware_id = hostname
        self._dev_stat.message = 'No Data'
        self._dev_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                  KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._last_temp_time = 0
        self._last_usage_time = 0
        self._last_nfs_time = 0
        self._last_publish_time = 0
        self._last_bat_time = 0
        self._last_robot_time = 0
	self._last_net_time = 0
	self._last_dev_time = 0

        ##@todo Need wireless stuff, at some point, put NFS in usage status
        
        # Start checking everything
        self.check_temps()
        if self._check_nfs:
            self.check_nfs_stat()
        self.check_usage()
        self.check_bat()
        self.check_net()	
        self.check_dev()	
        self.check_robot()
    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temps_timer:
            self._temps_timer.cancel()

        if self._nfs_timer:
            self._nfs_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

	if self._bat_timer:
            self._bat_timer.cancel()

	if self._robot_timer:
            self._robot_timer.cancel()

	if self._net_timer:
            self._net_timer.cancel()

	if self._dev_timer:
            self._dev_timer.cancel()

    def check_nfs_stat(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return

        nfs_level = 0
        msg = 'OK'
        vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                 KeyValue(key = 'Time Since Last Update', value = str(0) )]

        try:
            p = subprocess.Popen('iostat -n',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            
            if retcode != 0:
                nfs_level = DiagnosticStatus.ERROR
                msg = 'iostat Error'
                vals.append(KeyValue(key = '\"iostat -n\" Call Error'))
                stdout = ''
                

            for index, row in enumerate(stdout.split('\n')):
                if index < 3:
                    continue
                
                lst = row.split()
                if len(lst) < 7:
                    continue
                
                file_sys = lst[0]
                read_blk = lst[1]
                write_blk = lst[2]
                read_blk_dir = lst[3]
                write_blk_dir = lst[4]
                r_blk_srv = lst[5]
                w_blk_srv = lst[6]
                
                vals.append(KeyValue(
                        key = '%s Read Blks/s' % file_sys, value=read_blk))
                vals.append(KeyValue(
                        key = '%s Write Blks/s' % file_sys, value=write_blk))
                vals.append(KeyValue(
                        key = '%s Read Blk dir/s' % file_sys, value=read_blk_dir))
                vals.append(KeyValue(
                        key = '%s Write Blks dir/s' % file_sys, value=write_blk_dir))
                vals.append(KeyValue(
                        key = '%s Read Blks srv/s' % file_sys, value=r_blk_srv))
                vals.append(KeyValue(
                        key = '%s Write Blks srv/s' % file_sys, value=w_blk_srv))
                
        except Exception, e:
            rospy.logerr(traceback.format_exc())
            nfs_level = DiagnosticStatus.ERROR
            msg = 'Exception'
            vals.append(KeyValue(key = 'Exception', value = str(e)))
          
        with self._mutex:
            self._nfs_stat.level = nfs_level
            self._nfs_stat.message = msg
            self._nfs_stat.values = vals
            
            self._last_nfs_time = rospy.get_time()
            
            if not rospy.is_shutdown():
                self._nfs_timer = threading.Timer(10.0, self.check_nfs_stat)
                self._nfs_timer.start()
            else:
                self.cancel_timers()


    ## Call every 10sec at minimum
    def check_temps(self):
        if rospy.is_shutdown():
            self._mutex.acquire()
            self.cancel_timers()
            self._mutex.release()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = str(0) ) ]
        diag_msgs = []
        diag_level = 0

        if self._check_ipmi:
            ipmi_vals, ipmi_msgs, ipmi_level = check_ipmi()
            diag_vals.extend(ipmi_vals)
            diag_msgs.extend(ipmi_msgs)
            diag_level = max(diag_level, ipmi_level)

        if self._check_core_temps:
            core_vals, core_msgs, core_level = check_core_temps(self._temp_vals)
            diag_vals.extend(core_vals)
            diag_msgs.extend(core_msgs)
            diag_level = max(diag_level, core_level)

        clock_vals, clock_msgs, clock_level = check_clock_speed(self._enforce_speed)
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

 
        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = ', '.join(diag_log)
        else:
            message = stat_dict[diag_level]

        with self._mutex:
            self._last_temp_time = rospy.get_time()
            
            self._temp_stat.level = diag_level
            self._temp_stat.message = message
            self._temp_stat.values = diag_vals
            
            if not rospy.is_shutdown():
                self._temp_timer = threading.Timer(10.0, self.check_temps)
                self._temp_timer.start()
            else:
                self.cancel_timers()

# creation of the Battery tab

    def check_bat(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        
        # Check Battery
        bat_level, bat_vals = check_bat()
        diag_vals.extend(bat_vals)
        diag_level = max(diag_level, bat_level)

	
        # Update status
        with self._mutex:
            self._last_bat_time = rospy.get_time()
            self._bat_stat.level = diag_level
            self._bat_stat.values = diag_vals
            
            self._bat_stat.message = stat_dict[diag_level]
            
            if not rospy.is_shutdown():
                self._bat_timer = threading.Timer(10.0, self.check_bat)
                self._bat_timer.start()
            else:
                self.cancel_timers()

# creation of the Robot tab

    def check_robot(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        
        # Check Battery
        robot_level, robot_vals = check_robot()
        diag_vals.extend(robot_vals)
        diag_level = max(diag_level, robot_level)

	
        # Update status
        with self._mutex:
            self._last_robot_time = rospy.get_time()
            self._robot_stat.level = diag_level
            self._robot_stat.values = diag_vals
            
            self._robot_stat.message = stat_dict[diag_level]
            
            if not rospy.is_shutdown():
                self._robot_timer = threading.Timer(10.0, self.check_robot)
                self._robot_timer.start()
            else:
                self.cancel_timers()

#creation of the Network tab

    def check_net(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        
        # Check Network
        net_level, net_vals = check_net()
        diag_vals.extend(net_vals)
        diag_level = max(diag_level, net_level)

        # Update status
        with self._mutex:
            self._last_net_time = rospy.get_time()
            self._net_stat.level = diag_level
            self._net_stat.values = diag_vals
            
            self._net_stat.message = stat_dict[diag_level]
            
            if not rospy.is_shutdown():
                self._net_timer = threading.Timer(10.0, self.check_net)
                self._net_timer.start()
            else:
                self.cancel_timers()

# creation of the device tab

    def check_dev(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        
        # Check Device
        dev_level, dev_vals = check_dev()
        diag_vals.extend(dev_vals)
        diag_level = max(diag_level, dev_level)

	
        # Update status
        with self._mutex:
            self._last_dev_time = rospy.get_time()
            self._dev_stat.level = diag_level
            self._dev_stat.values = diag_vals
            
            self._dev_stat.message = stat_dict[diag_level]
            
            if not rospy.is_shutdown():
                self._dev_timer = threading.Timer(10.0, self.check_dev)
                self._dev_timer.start()
            else:
                self.cancel_timers()

    def check_usage(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        
        # Check mpstat
        mp_level, mp_vals = check_mpstat()
        diag_vals.extend(mp_vals)
        diag_level = max(diag_level, mp_level)
            
        # Check uptime
        uptime_level, up_vals = check_uptime()
        diag_vals.extend(up_vals)
        diag_level = max(diag_level, uptime_level)
        
        # Check memory
        mem_level, mem_vals = check_memory()
        diag_vals.extend(mem_vals)
        diag_level = max(diag_level, mem_level)
            
       

        # Update status
        with self._mutex:
            self._last_usage_time = rospy.get_time()
            self._usage_stat.level = diag_level
            self._usage_stat.values = diag_vals
            
            self._usage_stat.message = stat_dict[diag_level]
            
            if not rospy.is_shutdown():
                self._usage_timer = threading.Timer(10.0, self.check_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def publish_stats(self):
        with self._mutex:
            # Update everything with last update times
            update_status_stale(self._temp_stat, self._last_temp_time)
            update_status_stale(self._usage_stat, self._last_usage_time)
	    update_status_stale(self._bat_stat, self._last_bat_time)
	    update_status_stale(self._robot_stat, self._last_robot_time)
	    update_status_stale(self._net_stat, self._last_net_time)
	    update_status_stale(self._dev_stat, self._last_dev_time)
            if self._check_nfs:
                update_status_stale(self._nfs_stat, self._last_nfs_time)

            msg = DiagnosticArray()
            msg.header.stamp = rospy.get_rostime()
            msg.status.append(self._temp_stat)
            msg.status.append(self._usage_stat)
	    msg.status.append(self._bat_stat)
	    msg.status.append(self._robot_stat)
	    msg.status.append(self._net_stat)
	    msg.status.append(self._dev_stat)
            if self._check_nfs:
                msg.status.append(self._nfs_stat)

            if rospy.get_time() - self._last_publish_time > 0.5:
                self._diag_pub.publish(msg)
                self._last_publish_time = rospy.get_time()


if __name__ == '__main__':
    hostname = socket.gethostname()

    import optparse
    parser = optparse.OptionParser(usage="usage: cpu_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default = hostname)
    options, args = parser.parse_args(rospy.myargv())

    try:
        #rospy.init_node('cpu_monitor_%s' % hostname)
	rospy.init_node('diagnostic')
	RFlag_pub = rospy.Publisher('Run_flag', std_msgs.msg.Int32)
	Robot_Voltage_pub = rospy.Publisher('Robot_voltage', std_msgs.msg.Float32)
	PC_Voltage_pub = rospy.Publisher('PC_voltage', std_msgs.msg.Float32)
    except rospy.exceptions.ROSInitException:
        print 'CPU monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)

    cpu_node = CPUMonitor(hostname, options.diag_hostname)

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            cpu_node.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        rospy.logerr(traceback.format_exc())

    cpu_node.cancel_timers()
    sys.exit(0)
    


    

            

