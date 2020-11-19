#!/usr/bin/env python
import rospy
PKG = 'ilocator_control'
import roslib; roslib.load_manifest(PKG)
import rospkg
from geometry_msgs.msg  import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import os
from utils.pure_pursuit import purePursuitController
from robot_localization.srv import *
from geographic_msgs.msg import GeoPoint

class ilocatorbot():
	def __init__(self):
	    #Creating our node,publisher and subscriber.
	    rospy.init_node('ilocatorbot_controller', anonymous=True)
	    self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	    self.control_status_publisher = rospy.Publisher('control_status',String,queue_size=10)
	    self.pose_subscriber = rospy.Subscriber('/odometry/filtered_map', Odometry, self.callback)
	    self.robot_pos = Odometry()

	    # Set up the velocity and lookahead distance.
	    self.velocity = 0.3
	    self.lookahead = 0.5

	    # Read the path.
	    self.path = []
	    self.pathFileName = os.path.join(rospkg.RosPack().get_path('ilocator_control'),'data/path.csv')
	    self.loadPath()

	    # Set up the rate.
	    self.rate = rospy.Rate(10)

    def loadPath(self):
        lat_long_points = np.genfromtxt(self.pathFileName, delimiter = ',')
        rospy.wait_for_service('fromLL')
        transformation_method = rospy.ServiceProxy('fromLL', FromLL)
        try:
            for gps_point in lat_long_points:
                lat_long = GeoPoint()
                lat_long.latitude = gps_point[0]
                lat_long.longitude = gps_point[1]
                lat_long.altitude = 0.0
                response = transformation_method(lat_long) # maybe we need to pass a special message
                self.path.append([response.map_point.x,response.map_point.y])
            self.control_status_publisher.publish('Path loaded')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


	#Callback function implementing the pose value received
	def callback(self, data):
	    self.robot_pos = data
	    self.robot_pos.pose.pose.position.x = round(self.robot_pos.pose.pose.position.x, 4)
	    self.robot_pos.pose.pose.position.y = round(self.robot_pos.pose.pose.position.y, 4)


	def run(self):
		p1 = 0
		p2 = 1
		while p2 < len(self.path):
			robot_pos = [self.robot_pos.pose.pose.position.x, self.robot_pos.pose.pose.position.y, self.robot_pos.pose.pose.orientation.x]

			start_point = self.path[p1]
			end_point = self.path[p2]
			v,omega = purePursuitController(start_point, end_point, self.velocity, robot_pos,self.lookahead)
			# v,omega = self.pidController(end_point, self.velocity, robot_pos)
			vel_msg = Twist()

			#linear velocity in the x-axis:
			vel_msg.linear.x = self.velocity
			vel_msg.linear.y, vel_msg.linear.z = 0,0

			#angular velocity in the z-axis:
			vel_msg.angular.z = omega
			vel_msg.angular.x, vel_msg.angular.y = 0,0

			#Publishing our vel_msg
			self.velocity_publisher.publish(vel_msg)
			self.control_status_publisher.publish('On path.')
			self.rate.sleep()

			# If at the goal point, change the points fed to the controller.
			if np.linalg.norm(robot_pos[:2] - self.path[p2]) < 0.1:
				p1 += 1
				p2 += 1

		# When finished, stop the robot.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		self.control_status_publisher.publish('Finished at goal point.')
		rospy.spin()




if __name__ == '__main__':
	try:
	    x = ilocatorbot()
	    x.run()
	except rospy.ROSInterruptException: pass