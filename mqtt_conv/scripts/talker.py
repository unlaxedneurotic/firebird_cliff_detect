#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
from io import BytesIO as StringIO
import paho.mqtt.client as mqtt

#mqttc = paho.Client()
global msg_str,pub

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe('robobits/test')  # Subscribe to the topic “digitest/test1”, receive any messages published on it


def on_message(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
    global pub
    print("Message received-> " + msg.topic + " " + str(msg.payload))  # Print a received msg
    msg_str=str(msg.payload)
    twist_msg= Twist()
    try:
        msg_arr=json.loads(msg_str)
        twist_msg.linear.x=msg_arr['twist_message']['linear']['x']
        twist_msg.angular.z=msg_arr['twist_message']['angular']['z']
    except:
        rospy.loginfo("Invalid MQTT message")
    #print("serialized-> " + json.dumps(twist_msg))  # Print a received msg
    rospy.loginfo(twist_msg)
    strg= StringIO()
    #twist_msg.serialize(strg)
    #rospy.loginfo(strg)
    print(strg.getvalue())
    #rospy.loginfo(twist_msg.dese())
    pub.publish(twist_msg)


def talker():
    global pub
    #pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    pub = rospy.Publisher('/ros0xrobot/mqtt_vel', Twist, queue_size=10)
    rospy.init_node('digi_mqtt_test')
    rate = rospy.Rate(10) # 10hz
    print("On") 
    client = mqtt.Client("digi_mqtt_test")  # Create instance of client with client ID “digi_mqtt_test”
    client.on_connect = on_connect  # Define callback function for successful connection
    client.on_message = on_message  # Define callback function for receipt of a message
    #client.connect("localhost", 1883, 60)  # Connect to (broker, port, keepalive-time)
    client.connect("broker.hivemq.com", 1883, 60)  # Connect to (broker, port, keepalive-time)
    #client.connect("broker.hivemq.com", 1883, 60)  # Connect to (broker, port, keepalive-time)
    #client.connect('127.0.0.1', 17300)
    client.loop_forever()  # Start networking daemon

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
