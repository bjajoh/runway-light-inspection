#!/usr/bin/env/python
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

def lightingNode():
    rospy.init_node('lighting')
    rospy.Subscriber("state", String,stateCallback)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        switch(incomingState){
            case "api-fail":
                pixels.fill((255,0,255))
                break
            case "ready":
                pixels.fill((0, 255, 0))
                break
            case "db-down-fail":
                pixels.fill((255,0,255))
                break
            case "driving":
                pixels.fill((255, 255, 0))
                break
            case "emergency":
                pixels.fill(255, 0, 0))
                break
            case "finished":
                pixels.fill((0,0,255))
                break
            case "dp-up-fail":
                pixels.fill((255,0,255))
                break
            default:
                pass()
        }

        rospy.spin()
