#!/usr/bin/python3
import rospy
import board
import neopixel
import time


pixels = neopixel.NeoPixel(board.D18, 46)
incomingState = None

#def stateCallback(data):
#    global incomingState
#    incomingState = data.data

def assi():
    global incomingState
    rospy.init_node('lighting')
#    rospy.Subscriber("state", String,stateCallback)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pixels.fill((255,0,0))
        time.sleep(0.1)
        pixels.fill((0,0,0))
        time.sleep(0.5)
        pixels.fill((255,0,0))
        time.sleep(0.1)
        pixels.fill((0,0,0))
        time.sleep(0.5)
        rospy.spin()

if __name__ == '__main__':
    try:
        assi()
#        os._exit(os.EX_OK)
    except rospy.ROSInterruptException:
        pass
