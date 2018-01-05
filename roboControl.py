#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import aquashoko
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from aquashoko_ros.cfg import AquashokoJointControlParamsConfig
from dynamic_reconfigure.server import Server
import copy
config_start = False
yaw1_pid = aquashoko.AQUASHOKO_PID()
pitch2_pid = aquashoko.AQUASHOKO_PID()
pitch3_pid = aquashoko.AQUASHOKO_PID()

yaw1_pid.kp = 10
yaw1_pid.ki = 0.2
yaw1_pid.kd = 2.5
yaw1_pid.upplim = 10.0
yaw1_pid.lowlim = -10.0

pitch2_pid.kp = 2.0
pitch2_pid.ki = 1.0
pitch2_pid.kd = 0.25
pitch2_pid.upplim = 10.0
pitch2_pid.lowlim = -10.0

pitch3_pid.kp = 2.0
pitch3_pid.ki = 1.0
pitch3_pid.kd = 0.25
pitch3_pid.upplim = 10.0
pitch3_pid.lowlim = -10.0

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.name)
    aquashoko.aquashoko_set(0,0,data.position[0])
    aquashoko.aquashoko_set(0,1,data.position[1])
    aquashoko.aquashoko_set(0,2,data.position[2])
    aquashoko.aquashoko_set(1,0,data.position[3])
    aquashoko.aquashoko_set(1,1,data.position[4])
    aquashoko.aquashoko_set(1,2,data.position[5])
    aquashoko.aquashoko_set(2,0,data.position[6])
    aquashoko.aquashoko_set(2,1,data.position[7])
    aquashoko.aquashoko_set(2,2,data.position[8])
    aquashoko.aquashoko_set(3,0,data.position[9])
    aquashoko.aquashoko_set(3,1,data.position[10])
    aquashoko.aquashoko_set(3,2,data.position[11])
    aquashoko.aquashoko_put()

def listener():
    global yaw1_pid, pitch2_pid, pitch3_pid
    aquashoko.aquashoko_init()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('aquashoko_listener', anonymous=False)

    rospy.Subscriber('aquashoko_chatter', JointState, callback)
    joint_state_pub = rospy.Publisher('aquashoko_joint_positions', JointState, queue_size = 1)
    joint_msg = JointState()
    joint_positions = [0,0,0, 0,0,0, 0,0,0, 0,0,0]

    cfg_server = Server(AquashokoJointControlParamsConfig, cfg_callback)

    aquashoko.aquashoko_set_pid(0, yaw1_pid)
    aquashoko.aquashoko_set_pid(1, pitch2_pid)
    aquashoko.aquashoko_set_pid(2, pitch3_pid)
            

    for i in range(5):
        aquashoko.aquashoko_put_pids()
        rospy.sleep(0.1)
        # this callback should return config data back to server

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        aquashoko.aquashoko_pull();
        joint_msg.header.stamp = rospy.Time.now()
        
        #n = 0
        #for i in range(4):
        #    for j in range(3):
        #        joint_positions[n*i+j]  = aquashoko.aquashoko_get(i,j)
        #    n = n + 1 
        joint_positions[0]  = aquashoko.aquashoko_get(0,0)
        joint_positions[1]  = aquashoko.aquashoko_get(0,1)
        joint_positions[2]  = aquashoko.aquashoko_get(0,2)
        joint_positions[3]  = aquashoko.aquashoko_get(1,0)
        joint_positions[4]  = aquashoko.aquashoko_get(1,1)
        joint_positions[5]  = aquashoko.aquashoko_get(1,2)
        joint_positions[6]  = aquashoko.aquashoko_get(2,0)
        joint_positions[7]  = aquashoko.aquashoko_get(2,1)
        joint_positions[8]  = aquashoko.aquashoko_get(2,2)
        joint_positions[9]  = aquashoko.aquashoko_get(3,0)
        joint_positions[10]  = aquashoko.aquashoko_get(3,1)
        joint_positions[11]  = aquashoko.aquashoko_get(3,2)

        joint_msg.position = copy.deepcopy(joint_positions)
        joint_state_pub.publish(joint_msg)
        #print aquashoko.aquashoko_get(0,1)
        #rospy.spin()

def cfg_callback(config, level):
        global config_start, yaw1_pid, pitch2_pid, pitch3_pid

        if not config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.yaw1_kp = yaw1_pid.kp
            config.yaw1_ki = yaw1_pid.ki
            config.yaw1_kd = yaw1_pid.kd
            config.yaw1_upplim = yaw1_pid.upplim
            config.yaw1_lowlim = yaw1_pid.lowlim

            config.pitch2_kp = pitch2_pid.kp
            config.pitch2_ki = pitch2_pid.ki
            config.pitch2_kd = pitch2_pid.kd
            config.pitch2_upplim = pitch2_pid.upplim
            config.pitch2_lowlim = pitch2_pid.lowlim

            config.pitch3_kp = pitch3_pid.kp
            config.pitch3_ki = pitch3_pid.ki
            config.pitch3_kd = pitch3_pid.kd
            config.pitch3_upplim = pitch3_pid.upplim
            config.pitch3_lowlim = pitch3_pid.lowlim
            
            config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            yaw1_pid.kp = config.yaw1_kp
            yaw1_pid.ki = config.yaw1_ki
            yaw1_pid.kd = config.yaw1_kd
            yaw1_pid.upplim = config.yaw1_upplim 
            yaw1_pid.lowlim = config.yaw1_lowlim 

            pitch2_pid.kp = config.pitch2_kp
            pitch2_pid.ki = config.pitch2_ki
            pitch2_pid.kd = config.pitch2_kd
            pitch2_pid.upplim = config.pitch2_upplim
            pitch2_pid.lowlim = config.pitch2_lowlim

            pitch3_pid.kp = config.pitch3_kp
            pitch3_pid.ki = config.pitch3_ki
            pitch3_pid.kd = config.pitch3_kd
            pitch3_pid.upplim = config.pitch3_upplim
            pitch3_pid.lowlim = config.pitch3_lowlim
            
            aquashoko.aquashoko_set_pid(0, yaw1_pid)
            aquashoko.aquashoko_set_pid(1, pitch2_pid)
            aquashoko.aquashoko_set_pid(2, pitch3_pid)
            aquashoko.aquashoko_put_pids()

            #for i in range(10):
            #    aquashoko.aquashoko_put_pids()
        # this callback should return config data back to server
        return config

if __name__ == '__main__':
    listener()
