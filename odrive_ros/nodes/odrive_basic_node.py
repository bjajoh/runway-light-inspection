#!/usr/bin/env python3
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
import numpy as np

from odrive_interface import ODriveInterfaceAPI, ODriveFailure
from odrive_interface import ChannelBrokenException, ChannelDamagedException
from odrive_simulator import ODriveInterfaceSimulator


class ODriveNode(object):


        def __init__(self):

            self.wheel_track = 0.4
            self.m_s_to_value = 1.0
            self.tyre_circumference = 0.2032
            self.angular_vel_limit = 2.0
            self.vel_subscriber = rospy.Subscriber("/cmd_vel",Twist, self.cmd_vel_callback, queue_size=2)
            self.status_pub = rospy.Publisher('/odrive_basic_node/status', std_msgs.msg.String, queue_size=2)
            self.encoder_pub = rospy.Publisher('/odrive_basic_node/twist_estimation',Twist, queue_size=10)
            self.status = "disconnected"
            self.interface = ODriveInterfaceAPI()


        def connect(self):
            self.status_pub.publish("Connecting...")
            result = self.interface.connect()
            result = self.interface.connect()
            if result == True:
                self.status = "connected"
            else:
                self.status = "not connected"
            self.status_pub.publish(("Connecting... result: {}".format(self.status)))
            


        def engage(self):
                result = self.interface.engage()
                self.status_pub.publish("Engaging... result: {}".format(result))


        def release(self):
                self.interface.release()
                self.status_pub.publish("Released")


        def compute_estimated_twist(self, left_angular_vel, right_angular_vel):
                message = Twist()
                left_linear_vel = -1.0*left_angular_vel*self.tyre_circumference*2.0
                right_linear_vel = right_angular_vel*self.tyre_circumference*2.0
                angular_vel = (right_linear_vel-left_linear_vel)/self.wheel_track*2.0
                linear_vel = (left_linear_vel+right_linear_vel)/2.0
                message.linear.x = linear_vel
                message.angular.z = angular_vel
                return message
                    

        def convert(self, forward, ccw):
                angular_to_linear = ccw * (self.wheel_track/2.0) 
                left_angular_vel  = (forward - angular_to_linear/2.0) * self.m_s_to_value/self.tyre_circumference/2.0
                right_angular_vel = (forward + angular_to_linear/2.0) * self.m_s_to_value/self.tyre_circumference/2.0
                left_angular_vel = np.sign(left_angular_vel) * min(abs(left_angular_vel),self.angular_vel_limit)
                right_angular_vel = np.sign(right_angular_vel) * min(abs(right_angular_vel),self.angular_vel_limit)
                return left_angular_vel,right_angular_vel


        def cmd_vel_callback(self, msg):
                left_angular_vel, right_angular_vel = self.convert(msg.linear.x, msg.angular.z)
                self.interface.drive(right_angular_vel,left_angular_vel)
                self.status_pub.publish("Driving... velocities: {},{}".format(left_angular_vel,right_angular_vel))


        def encoder_publisher_loop(self):
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                try:
                    left_vel_estimate = self.interface.left_vel_estimate()
                    right_vel_estimate = self.interface.right_vel_estimate()
                    print("encoder estimations: {},{}".format(left_vel_estimate,right_vel_estimate))
                    left_pos = str(self.interface.left_pos())
                    right_pos = str(self.interface.right_pos())
                    left_current = str(self.interface.left_current())
                    right_current = str(self.interface.right_current())
                    message = self.compute_estimated_twist(left_vel_estimate, right_vel_estimate)
                    self.encoder_pub.publish(message)
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break



def stopEnginesAndRelease():
    global odrive_node
    odrive_node.interface.drive(0,0)
    odrive_node.release()

def start_odrive(odrive_node):
        rospy.init_node('odrive')
        odrive_node.connect()
        odrive_node.engage()
        odrive_node.encoder_publisher_loop()
        # odrive_node.main_loop()
        rospy.on_shutdown(stopEnginesAndRelease)
        rospy.spin()    
        
if __name__ == '__main__':
        try:
                odrive_node = ODriveNode()
                start_odrive(odrive_node)
        except rospy.ROSInterruptException:
                pass
