#!/usr/bin/python

from __future__ import print_function

import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import message_filters


class histogram_equalize:

    def __init__(self, approximate_sync=True):

        self.left_subscriber = message_filters.Subscriber("/left/image_rect",Image, queue_size = 1000)
        self.right_subscriber = message_filters.Subscriber("/right/image_rect",Image, queue_size = 1000)

        self.left_image_pub = rospy.Publisher("/left_hist/image_rect",Image, queue_size=1000)
        self.right_image_pub = rospy.Publisher("/right_hist/image_rect",Image, queue_size=1000)

        # for speedo1 Lake Jocassee: clipLimit=1.5, titleGridSize=(2,2)
        # for stereo rig2 Florida cave: don't use this node
        # for stereo Rig Bus: clipLimit=2.0, tileGridSize=(5, 5)
        # these parameters can be changed to obtain best results for a particluar package

        # For more documentation see: https://docs.opencv.org/3.1.0/d5/daf/tutorial_py_histogram_equalization.html

        self.histenhance = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(5,5))

        Synchronizer = message_filters.ApproximateTimeSynchronizer if approximate_sync else message_filters.TimeSynchronizer
        slop = {1} if approximate_sync else {}
        self.time_sync = Synchronizer([self.left_subscriber,self.right_subscriber], 100, *slop)

        self.time_sync.registerCallback(self.img_callback)

    def img_callback(self,left_msg,right_msg):

        #rospy.loginfo(" Received Image message")
        ## image msg to CV image##
    
        left_image_cv = bridge.imgmsg_to_cv2(left_msg, "mono8")
        right_image_cv = bridge.imgmsg_to_cv2(right_msg, "mono8")


        ##Contrast Enhancement ##
        left_enhanced_img = self.histenhance.apply(left_image_cv)     
        right_enhanced_img = self.histenhance.apply(right_image_cv)
      
        #print (type(left_enhanced_img))


        #### Create Image Msg ####
        enhanced_left_msg = bridge.cv2_to_imgmsg(left_enhanced_img, "mono8")
        enhanced_left_msg.header.stamp = left_msg.header.stamp
        enhanced_left_msg.height = left_msg.height
        enhanced_left_msg.width = left_msg.width
        enhanced_right_msg = bridge.cv2_to_imgmsg(right_enhanced_img, "mono8")
        enhanced_right_msg.header.stamp = right_msg.header.stamp
        enhanced_right_msg.height = right_msg.height
        enhanced_right_msg.width = right_msg.width


        # Publish new image
        self.left_image_pub.publish(enhanced_left_msg)
        self.right_image_pub.publish(enhanced_right_msg)


if __name__ == "__main__":
    approximate_sync = rospy.get_param("~approximate_sync", True)
    histogram_equalize = histogram_equalize(approximate_sync)
    bridge = CvBridge()
    rospy.init_node('histogram_equalize', anonymous=True)
    while(not rospy.is_shutdown()):
        rospy.spin()
