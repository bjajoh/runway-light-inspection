#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

def fix_imu (msg):
    msg.header.stamp = rospy.Time.now()
    pub_imu.publish(msg)

def fix_twist (msg):
    msg.header.stamp = rospy.Time.now()
    msg.twist.twist.angular.z = msg.twist.twist.angular.z/2
    pub_twist.publish(msg)

def fix_navsat (msg):
    msg.header.stamp = rospy.Time.now()
    pub_navsat.publish(msg)

pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
pub_twist = rospy.Publisher('/odrive_basic_node/twist_estimation', TwistWithCovarianceStamped, queue_size=10)
pub_navsat = rospy.Publisher('/ublox_gps/fix', NavSatFix, queue_size=10)

rospy.init_node('fix_timestamp')

sub_imu = rospy.Subscriber ('/imu/data_old', Imu, fix_imu)
sub_twist = rospy.Subscriber ('/odrive_basic_node/twist_estimation_old', TwistWithCovarianceStamped, fix_twist)
sub_navsat = rospy.Subscriber ('/ublox_gps/fix_old', NavSatFix, fix_navsat)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    r.sleep()
