#!/usr/bin/env python
#import roslib; roslib.load_manifest('BINCADDY')
import rospy
#import roslib
import tf.transformations
import tf_conversions
import tf2_ros

import std_msgs.msg
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import std_srvs.srv

#roslib.load_manifest('diagnostic_updater')
import diagnostic_updater, diagnostic_msgs.msg

import time
import math
import traceback
import queue

from odrive_interface import ODriveInterfaceAPI, ODriveFailure
from odrive_interface import ChannelBrokenException, ChannelDamagedException
from odrive_simulator import ODriveInterfaceSimulator


class ODriveNode(object):


    def __init__(self):

    	self.wheel_track = 0.4
    	self.m_s_to_value = 1.0
    	self.tyre_circumference = 0.2032
    	self.vel_subscriber = rospy.Subscriber("/cmd_vel",Twist, self.cmd_vel_callback, queue_size=2)
    	self.status_pub = rospy.Publisher('status', std_msgs.msg.String, queue_size=2)
    	self.status = "disconnected"
    	self.interface = ODriveInterfaceAPI()


    def connect(self):
    	result = self.interface.connect()
    	result = self.interface.connect()
    	self.status = "connected"
    	self.status_pub.publish(("Connecting... result: {}".format(result)))


    def engage(self):
        result = self.interface.engage()
        self.status_pub.publish("Engaging... result: {}".format(result))


    def release(self):
        self.interface.release()
        self.status_pub.publish("Released")

    def convert(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0) 
        left_linear_val  = float((forward - angular_to_linear) * self.m_s_to_value)
        right_linear_val = float((forward + angular_to_linear) * self.m_s_to_value)
        print("Left: {}, Right: {}".format(left_linear_val,right_linear_val))
        return left_linear_val,right_linear_val


    def cmd_vel_callback(self, msg):
        left_linear_val, right_linear_val = self.convert(msg.linear.x, msg.angular.z)
        if left_linear_val < 4 and right_linear_val < 4:
            self.interface.drive(left_linear_val,right_linear_val)
            self.status_pub.publish("Driving... velocities: {},{}".format(left_linear_val,right_linear_val))



odrive_node = None

def stopEngines():
	global odrive_node
	release()

def start_odrive():
    global odrive_node
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    odrive_node.connect()
    odrive_node.engage()
    # odrive_node.main_loop()
    rospy.on_shutdown(stopEngines)	
    rospy.spin() 	


def release():
    global odrive_node
    odrive_node.release()
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        release()