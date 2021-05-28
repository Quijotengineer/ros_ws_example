#!/usr/bin/env python
"""
FishEye2CahillKeyesRosBridge takes in fisheye images from 4 topics and returns a 
Cahill-Keyes projection.
References:
[1]: http://wiki.ros.org/message_filters#Example_.28Python.29-1
[2]: https://answers.ros.org/question/311917/subscribing-from-2-cameras-simultaneously-in-sawyer-robot/
[3]: https://stackoverflow.com/questions/7587490/converting-numpy-array-to-opencv-array

"""

## OSD libraries
from __future__ import print_function # Must be at start of file
from py_perception_pkg import config_simulator as cs
from py_perception_pkg import config_depthMapper as dm
from py_perception_pkg import depthMapMsgGenerator as msgGen
from py_perception_pkg.msg import DepthMapMsg

## ROS libraries
# import roslib; roslib.load_manifest('py_perception_pkg') ## No longer needed https://answers.ros.org/question/115346/what-does-roslibload_manifest-do/
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
import message_filters # To process synced topics [1]
from cv_bridge import CvBridge, CvBridgeError
#from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

## Other libraries
import sys
import numpy as np
import cv2

class Fisheye2DepthmapRosBridge:

    def __init__(self):

        self.equi_num = 1

        ## Create publishers
        self.left_equi_pub = rospy.Publisher("/osd/equi_left",Image, queue_size=10)
        self.right_equi_pub = rospy.Publisher("/osd/equi_right",Image, queue_size=10)
        self.depth_map_pub = rospy.Publisher("/osd/depth_map", DepthMapMsg, queue_size=10)

        ## Create bridge between OpenCV and ROS images.
        self.bridge = CvBridge()

        ## Create subscribers
        self.image0_sub = message_filters.Subscriber("/osd/camera/image0",Image)
        self.image1_sub = message_filters.Subscriber("/osd/camera/image1",Image)
        self.image2_sub = message_filters.Subscriber("/osd/camera/image2",Image)
        self.image3_sub = message_filters.Subscriber("/osd/camera/image3",Image)

        ## Synchronise images with message_filter
        self.ts = message_filters.TimeSynchronizer([self.image0_sub, self.image1_sub,
                                                    self.image2_sub, self.image3_sub], queue_size=1)
        self.ts.registerCallback(self.fisheyeSync_cb)

        # Communicate instantiation
        rospy.loginfo("Instantiated Fisheye2DepthmapRosBridge object")

    def fisheyeSync_cb(self, image0_sub, image1_sub, image2_sub, image3_sub):
        
        ## Convert to OpenCV images to feed into model
        cv_image0 = self.bridge.imgmsg_to_cv2(image0_sub, "bgr8")
        cv_image1 = self.bridge.imgmsg_to_cv2(image1_sub, "bgr8")
        cv_image2 = self.bridge.imgmsg_to_cv2(image2_sub, "bgr8")
        cv_image3 = self.bridge.imgmsg_to_cv2(image3_sub, "bgr8")

        ## Feed into Borys' code to generate cahillkeyes
        equi_left = cs.convertSimulatedFisheye2Equi(cv_image0, cv_image1)
        equi_right = cs.convertSimulatedFisheye2Equi(cv_image3, cv_image2)

        ## Convert to OpenCV im for visualisation [3]
        cv_equi_left = cv2.cvtColor(equi_left, cv2.COLOR_RGB2BGR)
        cv_equi_right = cv2.cvtColor(equi_right, cv2.COLOR_RGB2BGR)

        ## Convert nd-array to image to publish
        self.left_equi_pub.publish(self.bridge.cv2_to_imgmsg(equi_left, "bgr8"))
        self.right_equi_pub.publish(self.bridge.cv2_to_imgmsg(equi_right, "bgr8"))

        log_str = "Published '{0}' equirectangular topics. '{1}'".format(self.equi_num, rospy.get_time())
        rospy.loginfo(log_str)
        self.equi_num += 1

        # DepthMap
        self.depth_map = dm.get_Depth(equi_left, equi_right)

        ## Convert depth_map from (1,208,80,5) to (208,80,5) array
        self.depth_map = self.depth_map.reshape((np.shape(self.depth_map)[1], np.shape(self.depth_map)[2], np.shape(self.depth_map)[3]))

        ## Display depth map images
        #self.imshowDepthMap(self.depth_map)

        ## Save images: need to normalise between 0 and 255
        #self.saveDepthMap(self.depth_map)

        ## Generate depth map message
        self.depth_map_msg = msgGen.DepthMapMsgGenerator(self.depth_map).genCustomDepthMapMsg()

        ## Publish depth_map_msg
        self.depth_map_pub.publish(self.depth_map_msg)

        ## Inform of callback execution
        #rospy.loginfo("Executed ckSubCallback at time: '{0}'".format(rospy.get_time()))
        #rospy.loginfo(self.depth_map_msg)
        rospy.loginfo("Sending depth_map_msg over depth_map topic")

def main():
    rospy.init_node('fisheye2equirectangular_node')
    Fisheye2DepthmapRosBridge()
    rate = rospy.Rate(3) # 3 Hz
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    main()
