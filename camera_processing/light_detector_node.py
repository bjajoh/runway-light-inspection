#!/usr/bin/env python
# license removed for brevity
import rospy

import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

###Gaussian blut on image
##blur = cv2.blur(img,(8,8))
##
##
#### hsv filter
##hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
##lights = cv2.inRange(hsv, (15,0,200), (180,255,255))
##
##
#### Cluster light hypothesis by using DBSCAN
##light_pixels = np.array(list(indices[:])).T
##db = DBSCAN(eps=5, min_samples=40).fit(light_pixels)
##
##core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
##core_samples_mask[db.core_sample_indices_] = True
##labels = db.labels_
##
### Number of clusters in labels, ignoring noise if present.
##n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
##n_noise_ = list(labels).count(-1)
##
##print('Estimated number of clusters: %d' % n_clusters_)
##print('Estimated number of noise points: %d' % n_noise_)
##
###plt.figure(figsize=(20,4))
###data = np.array(list(indices))
###xs = np.array(list(indices[1]))
###ys = np.array(list(indices[0]))
###plt.scatter(xs, ys, c=labels, cmap='jet')
###
###ax=plt.gca()                            # get the axis
###ax.set_ylim(ax.get_ylim()[::-1])        # invert the axis
###ax.xaxis.tick_top()                     # and move the X-Axis      
###
###plt.show()
##
##
#### Calculate single location per cluster buttom middle of each cluster as homography input
##light_locations =  np.zeros([0, 2])
##
##for cluster_id in range(np.max(labels)+1):
##    light_locations = np.vstack((light_locations, np.array([np.mean(xs[labels == [cluster_id]]), np.max(ys[labels == [cluster_id]])])))
##
###plt.figure(figsize=(20,4))
###plt.plot(light_locations[:,0], light_locations[:,1], 'rd')
###
###ax=plt.gca()                            # get the axis
###ax.set_ylim(ax.get_ylim()[::-1])        # invert the axis
###ax.xaxis.tick_top()                     # and move the X-Axis      
###
###plt.show()
##
##np.set_printoptions(threshold=100)
##print(light_locations)


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass