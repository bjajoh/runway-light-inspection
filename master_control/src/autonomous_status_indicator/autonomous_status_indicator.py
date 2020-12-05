#!/usr/bin/python3
import board
import neopixel
import time
import os
import time

import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, Int8
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped


pixels = neopixel.NeoPixel(board.D18, 46)
incomingState = None

#def stateCallback(data):
#    global incomingState
#    incomingState = data.data

timeout = 1 #seconds to switch state after no message

state = 0
path_state = 0
path_state_timestamp = time.time()
odrive_alive_timestamp = time.time()

# control status control_status_dict = {'on_path': 1, 'obtained_goal': 2}
def path_callback(msg):
    global path_state_timestamp
    path_state_timestamp = time.time()
    global path_state
    path_state = msg.data

def odrive_callback(msg):
    global odrive_alive_timestamp
    odrive_alive_timestamp = time.time()

def assi():
    global incomingState
    rospy.init_node('assi')

    path_subscriber = rospy.Subscriber("/controller/control_status_int",Int8, path_callback, queue_size=2)
    odrive_subscriber = rospy.Subscriber("/odrive_basic_node/twist_estimation", TwistWithCovarianceStamped, odrive_callback, queue_size=2)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        #print("odrive time", time.time() - odrive_alive_timestamp)
        if(time.time() - odrive_alive_timestamp < timeout):
            state = 1
            #print("path time", time.time() - path_state_timestamp)
            if(time.time() - path_state_timestamp < timeout):
                if(path_state == 1):
                    state = 2
                elif(path_state == 2):
                    state = 3
        else:
            state = 0

        if(state == 0):
            pixels.fill((0,0,0))
        elif(state == 1):
            pixels.fill((255,0,0))
        elif(state == 2):
            pixels.fill((255,0,0))
            time.sleep(0.1)
            pixels.fill((0,0,0))
            time.sleep(0.5)
            pixels.fill((255,0,0))
            time.sleep(0.1)
            pixels.fill((0,0,0))
            time.sleep(0.5)
        elif(state == 3):
            pixels.fill((0,0,255))
        elif(state == 4):
            pixels.fill((0,0,255))
            time.sleep(0.1)
            pixels.fill((0,0,0))
            time.sleep(0.5)
            pixels.fill((0,0,255))
            time.sleep(0.1)
            pixels.fill((0,0,0))
            time.sleep(0.5)
    rospy.spin()

if __name__ == '__main__':
    try:
        assi()
        os._exit(os.EX_OK)
    except rospy.ROSInterruptException:
        os._exit(os.EX_OK)
