robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://docs.ros.org/melodic/api/robot_localization/html/index.html

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
