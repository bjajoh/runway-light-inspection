# runway-light-inspection
## Mobile robot for automated visual inspection of the approach lighting system at Aarhus Airport (AAR). 
https://www.ilocator.com/proposal//cover.php?ProposalID=6ymasXDnLmC3NSEPzF7ANSlGMOQGCzl4Lx3Svz0prog&debug=yes

### Install
Install all dependencies:
```
git submodule update --init --recursive
```
```
rosdep install --from-paths src --ignore-src -r
```


Generate Path and correction data NEMA code https://nmeagen.org/

LEDs with Pi: https://learn.adafruit.com/neopixels-on-raspberry-pi/python-usage

Use catkin build instead of catkin_make: https://catkin-tools.readthedocs.io/en/latest/installing.html


### Running the camera
```
Starting and Stopping the ueye camera usb service:
sudo systemctl start ueyeusbdrc.service
sudo systemctl stop ueyeusbdrc.service
```
```
roslaunch common perception.launch
```
```
roslaunch common gnss.launch
```
```
roslaunch common rtk.launch
```
```
roslaunch common state_estimation.launch
```
``` 
roslaunch ueye mono.launch
``` 
``` 
roslaunch ueye stereo.launch
``` 

### Remote control the robot with the keyboard
```
sudo apt-get install ros-noetic-teleop-twist-keyboard

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Record Raw Data
Mono Camera:
```
rosbag record -a -x "/image_raw/theora(.*)|/image_raw/compressedDepth/(.*)|/image_raw/compressed(.*)"
```

Dual Camera:
```
rosbag record -a -x "/left/image_raw/theora(.*)|/left/image_raw/compressedDepth/(.*)|/left/image_raw/compressed(.*)|/right/image_raw/theora(.*)|/right/image_raw/compressedDepth/(.*)|/right/image_raw/compressed(.*)"
```

### Replay log data and removing recompute topics
```
rosbag play test.bag -l /odometry/filtered:=/trash /odometry/filtered_map:=/trash2 /odometry/gps:=/trash3 /tf:=/trash4 /tf_static:=/trash5 /initialpose:=/trash6 /gps/filtered:=/trash7 /rosout:=/trash8 /rosout_agg:=/trash9 /diagnostics:=/trash10
```

![Bilby Stampede](https://upload.wikimedia.org/wikipedia/en/b/b9/AAU_logo_2012.png)

![Bilby Stampede](https://s3-eu-west-1.amazonaws.com/businessautomation/Proposal_Full_Images/iLocator-GmbH_AAR_5774.jpg)
![Bilby Stampede](https://s3-eu-west-1.amazonaws.com/businessautomation/Proposal_Full_Images/iLocator-GmbH_inserts_5611.png)
