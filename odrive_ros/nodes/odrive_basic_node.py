#!/usr/bin/env python3
#import roslib; roslib.load_manifest('BINCADDY')
import rospy
#import roslib
import tf.transformations
import tf_conversions
import tf2_ros

import std_msgs.msg
from std_msgs.msg import Float64, Int32, Int8
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, TransformStamped
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
            self.tyre_circumference = 0.638
            self.angular_vel_limit = 10
            self.vel_subscriber = rospy.Subscriber("/cmd_vel",Twist, self.cmd_vel_callback, queue_size=2)
            self.emergency_obstacle_subscriber = rospy.Subscriber("/emergency_message",Int8, self.emergency_stop_callback, queue_size=2)
            self.status_pub = rospy.Publisher('/odrive_basic_node/status', std_msgs.msg.String, queue_size=2)
            self.status_pub_voltage = rospy.Publisher('/odrive_basic_node/bus_voltage', std_msgs.msg.String, queue_size=2)
            self.encoder_pub = rospy.Publisher('/odrive_basic_node/twist_estimation',TwistWithCovarianceStamped, queue_size=10)
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
                #self.interface.calibrate()
                result = self.interface.engage()
                self.status_pub.publish("Engaging... result: {}".format(result))


        def release(self):
                self.interface.drive(0,0)
                self.interface.release()
                self.status_pub.publish("Released")


        def compute_estimated_twist(self, left_angular_vel, right_angular_vel):
                message = TwistWithCovarianceStamped()
                left_linear_vel = -1.0*left_angular_vel*self.tyre_circumference
                right_linear_vel = right_angular_vel*self.tyre_circumference
                angular_vel = (right_linear_vel-left_linear_vel)/self.wheel_track
                linear_vel = (left_linear_vel+right_linear_vel)/2.0
                message.twist.twist.linear.x = linear_vel
                message.twist.twist.angular.z = angular_vel
                message.twist.covariance[0] = 0.1
                message.twist.covariance[7] = 0.1
                message.twist.covariance[14] = 0.1
                message.twist.covariance[21] = 0.1
                message.twist.covariance[28] = 0.1
                message.twist.covariance[35] = 0.1
                message.header.stamp = rospy.Time.now()
                # IN CASE WE NEED THE FRAME ID:
                message.header.frame_id = 'base_link'
                return message
                    

        def convert(self, forward, ccw):
                angular_to_linear = ccw * (self.wheel_track/2.0) 
                left_angular_vel  = (forward - angular_to_linear/2.0) * self.m_s_to_value/self.tyre_circumference
                right_angular_vel = (forward + angular_to_linear/2.0) * self.m_s_to_value/self.tyre_circumference
                left_angular_vel = np.sign(left_angular_vel) * min(abs(left_angular_vel),self.angular_vel_limit)
                right_angular_vel = np.sign(right_angular_vel) * min(abs(right_angular_vel),self.angular_vel_limit)
                return left_angular_vel,right_angular_vel


        def cmd_vel_callback(self, msg):
                left_angular_vel, right_angular_vel = self.convert(msg.linear.x, msg.angular.z)
                self.interface.drive(right_angular_vel,left_angular_vel)
                self.status_pub.publish("Driving... velocities: {},{}".format(left_angular_vel,right_angular_vel))

        def emergency_stop_callback(self):
                self.release()


        def encoder_publisher_loop(self):
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                try:
                    left_vel_estimate = self.interface.left_vel_estimate()
                    right_vel_estimate = self.interface.right_vel_estimate()
                    # print("encoder estimations: {},{}".format(left_vel_estimate,right_vel_estimate))
                    left_pos = str(self.interface.left_pos())
                    right_pos = str(self.interface.right_pos())
                    # left_current = str(self.interface.left_current())
                    # right_current = str(self.interface.right_current())
                    message = self.compute_estimated_twist(left_vel_estimate, right_vel_estimate)
                    self.encoder_pub.publish(message)
                    bus_voltage_message = str(self.interface.bus_voltage())
                    self.status_pub_voltage.publish(bus_voltage_message)
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
