#!/usr/bin/env python
import cv2
import numpy as np
import random
import rospy
import time

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointField, PointCloud2
import std_msgs
from std_msgs import msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import camera_demo_arucoTag_kinect as demo
import kinect_subscriber as kinect

TOPIC_NAME = 'aruco_transform'

#def get_dim(arr):
#    dim = [std_msgs.msg.MultiArrayDimension() for i in range(3)]
#    h = len(arr)
#    w = len(arr[0])
#
#    dim[0].label  = "height"
#    dim[0].size   = h
#    dim[0].stride = 3 * w * h
#    dim[1].label  = "width"
#    dim[1].size   = w
#    dim[1].stride = 3 * w
#    dim[2].label  = "channel"
#    dim[2].size   = 3
#    dim[2].stride = 3
#
#    return dim


if __name__ =="__main__":
    rospy.init_node("publisher")
    pub = rospy.Publisher(TOPIC_NAME, Float32MultiArray, queue_size=1)
    r = rospy.Rate(0.5)

    # let's build a 3x3 matrix:
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())

    mat.layout.dim[0].label = "height"
    mat.layout.dim[0].size = 3
    #mat.layout.dim[0].stride = 3*3
    mat.layout.dim[0].stride = 1

    mat.layout.dim[1].label = "width"
    mat.layout.dim[1].size = 3
    #mat.layout.dim[1].stride = 3
    mat.layout.dim[1].stride = 1

    #mat.layout.dim[2].label  = "channel"
    #mat.layout.dim[2].size   = 3
    ##mat.layout.dim[2].stride = 3
    #mat.layout.dim[2].stride = 1

    mat.layout.data_offset = 0
    mat.data = [0]*9

    # save a few dimensions:
    dstride0 = mat.layout.dim[0].stride
    dstride1 = mat.layout.dim[1].stride
    offset = mat.layout.data_offset

    while not rospy.is_shutdown():
        for i in range(3):
            for j in range(3):
                mat.data[offset + i + dstride1*j] = demo.cameraMatrix[i][j]

        print mat
        pub.publish(mat)
        r.sleep()


    while not rospy.is_shutdown():
        tmpmat = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                num = random.randrange(0,10)
                mat.data[offset + i + dstride1*j] = num
                tmpmat[i,j] = num
        #print tmpmat
        print mat
        pub.publish(mat)
        #rospy.loginfo("I'm sending:")
        #print tmpmat,"\r\n"
        r.sleep()



#if __name__ == '__main__':
#    rospy.init_node("publisher")
#    pub = rospy.Publisher(TOPIC_NAME, std_msgs.msg.Float32MultiArray, queue_size=1)
#
#    while not rospy.is_shutdown():
#        #data = [[0, 0, 0], [1, 1, 1]]
#        data = demo.cameraMatrix
#        dim = get_dim(data)
#
#        r = rospy.Rate(0.5)
#        # let's build a 3x3 matrix:
#        mat = std_msgs.msg.Float32MultiArray()
#        mat.layout.dim = dim
#        mat.layout.data_offset = 0
#        mat.data = data
#        #print mat
#
#
#        # save a few dimensions:
#        dstride0 = mat.layout.dim[0].stride
#        dstride1 = mat.layout.dim[1].stride
#        offset = mat.layout.data_offset
#
#        while not rospy.is_shutdown():
#            tmpmat = np.zeros((3,3))
#            for i in range(3):
#                for j in range(3):
#                    num = random.randrange(0,10)
#                    mat.data[offset + i + dstride1*j] = num
#                    tmpmat[i,j] = num
#            pub.publish(mat)
#            r.sleep()
#
#        #pub.publish(data)
#
#        time.sleep(1)
#
##    count = demo.START_COUNT
##    while not rospy.is_shutdown():
##        frame_current = kinect.get_rgb_image()
##        gray = frame_current.astype(np.uint8)
##
##        # get M_Cm0: camera to object pose
##        M_Cm0 = demo.get_M_Cm0(gray, gray)
##
##        dim = get_dim(M_Cm0)
##        layout = std_msgs.msg.MultiArrayLayout(dim, 0)
##        msg = std_msgs.msg.Float32MultiArray(layout, M_Cm0)
##
##        data = (msg.data,)
##        print data
##
##        #pub.publish(layout, msg.data)
##        #pub.publish(layout, data)
##        time.sleep(1)
#
