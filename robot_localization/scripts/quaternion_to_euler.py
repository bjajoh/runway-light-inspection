#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/imu/data', Imu, get_rotation)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()
