#!/usr/bin/env/python
import rospy
from datetime import datetime, timedelta, time
from std_msgs.msg import String
from ilocator_pythonLib import *

global isDone = 0
global kswitchStatus = 1
global lastKeepAlive 0
def trajCallback(data):
    #Quick callback function that just returns the value. Using this for the
    #subscriber stuff since new subscribed messages can be dropped in as arguments.
    if data.data == "finished":
        isDone = 1
def killswitchCallback(data):
    if lastKeepAlive == 0:
        lastKeepAlive = time(now)
    if data.data == "alive"
        kswitchStatus = 0

def stateNode():
    pub = rospy.Publisher('state', String, queue_size = 10)
    rospy.init_node('stateNode')
    rospy.Subscriber("killswitch", String, killswitchCallback)
    rospy.Subscriber("trajectory", String, trajCallback)
    rate = rospy.Rate(20)

    trajectoryState = ''
    hasAPIKey, successfulDBDownload, isAlive, successfulUpload, lockCycle = 0
    APIKey = ''

    ##Temp while we get killswitch ready
    isAlive = 1
    while not rospy.is_shutdown():
        lockCycle = 0

        while not lockCycle:
            isDone =
            if not hasAPIKey:
                rospy.loginfo("Acquiring API Key")
                lockCycle = 1
                APIKey = getAPIKey()
                rospy.loginfo("API key acquired: "+APIKey)
                if APIKey == '': #Error acquiring API Key
                    pub.publish("api-fail")
                else:
                    pub.publish("api-success")
                    hasAPIKey = 1

            if not successfulDBDownload:
                rospy.loginfo("Attempting database download")
                successfulDBDownload = getDBObjects(APIKey)
                if successfulDBDownload: #Error acquiring API Key
                    pub.publish("ready")
                    rospy.loginfo("Database downloaded, ready to drive.")
                else:
                    pub.publish("db-down-success")
                    rospy.loginfo("Database download failed.")
                lockCycle = 1

            if isAlive and not isDone:
                pub.publish("driving")
            if not isAlive:
                pub.publish("emergency")
                ros.loginfo("Emergency state entered, loss of keep alive signal.")

            if isDone:
                pub.publish("finished")
                if successfulUpload == 0:
                    successfulUpload = syncToDB(APIKey)
                    rospy.loginfo("Attempting database sync...")
                if successfulUpload:
                    rospy.loginfo("Successfulyy uploaded, run is complete.")

        rospy.spin()
