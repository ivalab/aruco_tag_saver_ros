#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time

import std_msgs
from std_msgs import msg
from sensor_msgs.msg import Image, PointField, PointCloud2
import sensor_msgs.point_cloud2 as pc2

import camera_demo_arucoTag_kinect as demo
import kinect_subscriber as kinect

TOPIC_NAME = 'aruco_transform'

def get_dim(arr):
    dim = [std_msgs.msg.MultiArrayDimension() for i in range(3)]
    h = len(arr)
    w = len(arr[0])

    dim[0].label  = "height"
    dim[0].size   = h
    dim[0].stride = 3 * w * h
    dim[1].label  = "width"
    dim[1].size   = w
    dim[1].stride = 3 * w
    dim[2].label  = "channel"
    dim[2].size   = 3
    dim[2].stride = 3

    return dim

def talker():
    pub = rospy.Publisher(TOPIC_NAME, std_msgs.msg.String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        pub.publish(hello_str)
    #    rate.sleep()

if __name__ == '__main__':
    talker()

#if __name__ == '__main__':
#    print(TOPIC_NAME)
#    pub = rospy.Publisher(TOPIC_NAME, std_msgs.msg.Float32MultiArray, queue_size=10)
#
##    rospy.init_node(TOPIC_NAME)
##    #while True:
##        data = demo.cameraMatrix
##        data = list(data)
##        dim = get_dim(data)
##
##        layout = std_msgs.msg.MultiArrayLayout(dim, 0)
##        #data = std_msgs.msg.Float32MultiArray(layout, data)
##
##        pub.publish(layout, data)
##        #pub.publish(layout, list(data.data))
#
#    count = demo.START_COUNT
#    while not rospy.is_shutdown():
#        frame_current = kinect.get_rgb_image()
#        gray = frame_current.astype(np.uint8)
#
#        # get M_Cm0: camera to object pose
#        M_Cm0 = demo.get_M_Cm0(gray, gray)
#
#        dim = get_dim(M_Cm0)
#        layout = std_msgs.msg.MultiArrayLayout(dim, 0)
#        msg = std_msgs.msg.Float32MultiArray(layout, M_Cm0)
#
#        data = (msg.data,)
#        print data
#
#        #pub.publish(layout, msg.data)
#        #pub.publish(layout, data)
#        time.sleep(1)
#
