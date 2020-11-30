#!/usr/bin/env python3
import rospy
PKG = 'path_tracking_controller'
import roslib; roslib.load_manifest(PKG)
import rospkg
import math
from geometry_msgs.msg  import Twist
from std_msgs.msg import String, Int8
from nav_msgs.msg import Odometry
import numpy as np
import os
from robot_localization.srv import *
from geographic_msgs.msg import GeoPoint
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)

def purePursuitController(p1, p2, velocity, robot_pos, l):	
	    epsilon = 0.0001
	    az = (p1[1]-p2[1])/(p1[0]-p2[0]+epsilon)
	    bz = p1[1]-(p1[1]-p2[1])/(p1[0]-p2[0]+epsilon)*p1[0]
	    dist_from_path = np.abs(az*robot_pos[0]-robot_pos[1]+bz+epsilon)/np.sqrt(az**2+1)
	    dist_to_goal = np.sqrt((robot_pos[0]-p2[0])**2+(robot_pos[1]-p2[1])**2)
	    if dist_from_path > l:
	        ac = -1.0/(az+epsilon)
	        bc = robot_pos[1]+(p1[0]-p2[0])/(p1[1]-p2[1])*robot_pos[0]
	        xc_num = (robot_pos[1]+(p1[0]-p2[0])/(p1[1]-p2[1]+epsilon)*robot_pos[0]-p1[1]+(p1[1]-p2[1])/(p1[0]-p2[0]+epsilon)*p1[0])
	        xc_den = (p1[1]-p2[1])/(p1[0]-p2[0]+epsilon) + (p1[0]-p2[0])/(p1[1]-p2[1]+epsilon)
	        xc = xc_num/xc_den
	        yc = ac*xc + bc
	        pursuit_point = [xc, yc]
	    else:
	        if dist_to_goal <= l:
	            pursuit_point = p2
	            l = dist_to_goal
	        else:
	            coeffs = [1+az**2, 2*az*bz-2*az*robot_pos[1]-2*robot_pos[0], 
	                      bz**2-2*bz*robot_pos[1]-l**2+robot_pos[0]**2+robot_pos[1]**2]
	            roots = np.real(np.roots(coeffs))
	            y = az*roots+bz
	            distance = np.sqrt((roots-p2[0])**2+(y-p2[1])**2)
	            min_root_index = np.argmin(distance)        
	            pursuit_point = [roots[min_root_index],y[min_root_index]]
	    translation_matrix = np.array([[np.cos(robot_pos[2]),-np.sin(robot_pos[2]), robot_pos[0]],
	                          [np.sin(robot_pos[2]), np.cos(robot_pos[2]), robot_pos[1]],
	                          [0, 0, 1]])
	    pursuit_point_matrix = np.expand_dims(np.transpose([pursuit_point[0],pursuit_point[1], 1]),axis=1)
	    robot_coord_matrix = np.matmul(np.linalg.inv(translation_matrix),pursuit_point_matrix)
	    curvature = 2*robot_coord_matrix[1]/l**2
	    omega = velocity*curvature
	    return velocity, omega


class ilocatorbot():
        def __init__(self):
            #Creating our node,publisher and subscriber.
                rospy.init_node('path_tracking_controller', anonymous=True)
                self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                self.control_status_publisher = rospy.Publisher('/controller/control_status',String,queue_size=10)
                self.control_status_publisher_int = rospy.Publisher('/controller/control_status_int',Int8,queue_size=10)
                self.pose_subscriber = rospy.Subscriber('/odometry/filtered_map', Odometry, self.callback)
                self.publisher = rospy.Publisher('/controller/visualization_marker', Marker, queue_size=10)
                self.robot_pos = Odometry()

            # Set up the velocity and lookahead distance.
                self.velocity = 0.3
                self.lookahead = 0.5
                self.goal_radius = 1.0

            # Set up the control status dict
                self.control_status_dict = {'on_path': 1, 'obtained_goal': 2}

            # Read the path.
                self.path = []
                self.pathFileName = os.path.join(rospkg.RosPack().get_path('path_tracking_controller'),'data/very_small_rectangle_cantine.csv')
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
                        self.path=np.array(self.path)
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
                        print("test")

                        get_rotation(self.robot_pos)

                        robot_pos = [self.robot_pos.pose.pose.position.x, self.robot_pos.pose.pose.position.y, yaw]

                        start_point = self.path[p1]
                        end_point = self.path[p2]

                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.type = marker.SPHERE
                        marker.action = marker.ADD
                        marker.scale.x = 1.1
                        marker.scale.y = 1.0
                        marker.scale.z = 1.0
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.pose.orientation.w = 1.0
                        marker.pose.position.x = start_point[0]
                        marker.pose.position.y = start_point[1]
                        marker.pose.position.z = 0

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
                        self.control_status_publisher_int.publish(self.control_status_dict['on_path'])
                        self.publisher.publish(marker)
                        self.rate.sleep()

                        # If at the goal point, change the points fed to the controller.
                        if np.linalg.norm(robot_pos[:2] - self.path[p2]) < self.goal_radius:
                                self.control_status_publisher.publish('Goal point changed to the next one.')
                                p1 += 1
                                p2 += 1

                # When finished, stop the robot.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.control_status_publisher.publish('Finished at goal point.')
            self.control_status_publisher_int.publish(self.control_status_dict['obtained_goal'])
            rospy.spin()




if __name__ == '__main__':
    try:
            x = ilocatorbot()
            x.run()
    except rospy.ROSInterruptException: pass
