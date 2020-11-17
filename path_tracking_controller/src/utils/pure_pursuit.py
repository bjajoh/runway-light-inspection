'''
Pure pursuit controller implementation.
Input consists of two points, the linear velocity of the robot, current robot position and the lookahead distance.
Returns angular velocity to obtain the goal point.
'''
import numpy as np


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