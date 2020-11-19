#!/usr/bin/env python3
import rospy
from datetime import datetime, timedelta, time
from std_msgs.msg import String
import ilocator_python_lib

isDone = 0
kswitchStatus = 1
lastKeepAlive = 0

def trajCallback(data):
    #Quick callback function that just returns the value. Using this for the
    #subscriber stuff since new subscribed messages can be dropped in as arguments.
    if data.data == "finished":
        isDone = 1
def killswitchCallback(data):
    if lastKeepAlive == 0:
        lastKeepAlive = time(now)
    if data.data == "alive":
        kswitchStatus = 0

def state_node():
    pub = rospy.Publisher('state', String, queue_size = 10)
    rospy.init_node('state_node')
    rospy.Subscriber("killswitch", String, killswitchCallback)
    rospy.Subscriber("trajectory", String, trajCallback)
    rate = rospy.Rate(20)

    trajectoryState = ''
    hasAPIKey =0
    successfulDBDownload =0
    isAlive =0
    successfulUpload =0
    lockCycle = 0
    APIKey = ''

    ##Temp while we get killswitch ready
    isAlive = 1
    while not rospy.is_shutdown():
        #lockCycle = 0

        while not rospy.is_shutdown():#lockCycle:
           # isDone = 0
            if not hasAPIKey:
                rospy.loginfo("Acquiring API Key")
                #lockCycle = 1
                APIKey = ilocator_pythonLib.getAPIKey()
                rospy.loginfo("API key acquired: "+APIKey)
                if APIKey == '': #Error acquiring API Key
                    pub.publish("api-fail")
                else:
                    pub.publish("api-success")
                    hasAPIKey = 1
            '''
            if not successfulDBDownload:
                rospy.loginfo("Attempting database download")
                successfulDBDownload = ilocator_pythonLib.getDBObjects(APIKey)
                if successfulDBDownload: #Error acquiring API Key
                    pub.publish("ready")
                    rospy.loginfo("Database downloaded, ready to drive.")
                else:
                    pub.publish("db-down-success")
                    rospy.loginfo("Database download failed.")
                #lockCycle = 1
            '''
            if isAlive and not isDone:
                pub.publish("driving")
                rospy.loginfo("We're driving now")
            if not isAlive:
                pub.publish("emergency")
                rospy.loginfo("Emergency state entered, loss of keep alive signal.")

            if isDone:
                pub.publish("finished")
                if successfulUpload == 0:
                    successfulUpload = ilocator_pythonLib.syncToDB(APIKey)
                    rospy.loginfo("Attempting database sync...")
                if successfulUpload:
                    rospy.loginfo("Successfulyy uploaded, run is complete.")

        rospy.spin()

state_node()
