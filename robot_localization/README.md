robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes.

The following example shows the ROS service call of the navsat_transform node. It can be used to transform WGS84 coordinates from and to local refference coordinates.

This example call "/fromLL" tansforms a point on AAU (Aalborg) campus east to the local coordinate system (meters).

``` 
rosservice call /fromLL "ll_point:
  latitude: 57.014319
  longitude: 9.987732
  altitude: 0" 
map_point: 
  x: 77.32918277148217
  y: -62.16242762274164
  z: 0.0
``` 

The following call does the oposite.

``` 
rosservice call /toLL "map_point:
  x: 1.0
  y: 1.0
  z: 0.0" 
ll_point: 
  latitude: 57.014300456711176
  longitude: 9.986101477639927
  altitude: 53.24186630672663
``` 
