#!/usr/bin/env/python3
import rospy
#These next two are for the LEDs, verify that i have the right ones.
#import board
#import neopixel #Assuming we can use this, I just went for something quick.
#Quick variable assignment for the pixels on the board. THIS WILL DEPEND ON THE
#LED STRIP. It's a slightly more filled in placeholder I guess
pixels = neopixel.NeoPixel(board.D18, 30)
global incomingState = ''


def stateCallback(data):
    incomingState = data.data

def lighting_node():
    rospy.init_node('lighting')
    rospy.Subscriber("state", String,stateCallback)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if incomingState == "api-fail":
            pixels.fill((255,0,255))
            break
        elif incomingState == "ready":
            pixels.fill((0, 255, 0))
            break
        elif incomingState == "db-down-fail":
            pixels.fill((255, 0, 255))
            break
        elif incomingState == "driving":
            pixels.fill((255,255,0))
            break
        elif incomingState == "emergency":
            pixels.fill((255, 0, 0))
            break
        elif incomingState == "finished":
            pixels.fill((0, 0, 255))
            break
        elif incomingState == "db-up-fail":
            pixels.fill((255, 0, 255))
            break
        else:
            pixels.fill((255, 255, 255))
            break
        rospy.spin()
