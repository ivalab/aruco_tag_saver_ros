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
HEIGHT = 4
WIDTH = 4


def get_dim():
    dim = [MultiArrayDimension() for i in range(2)]

    dim[0].label = "height"
    dim[0].size = HEIGHT
    dim[0].stride = 1

    dim[1].label = "width"
    dim[1].size = WIDTH
    dim[1].stride = 1

    return dim


if __name__ =="__main__":
    pub = rospy.Publisher(TOPIC_NAME, Float32MultiArray, queue_size=1)

    mat = Float32MultiArray()

    mat.layout.dim = get_dim()
    mat.layout.data_offset = 0

    mat.data = [0]*HEIGHT*WIDTH

    while not rospy.is_shutdown():
        frame_current = kinect.get_rgb_image()
        gray = frame_current.astype(np.uint8)
        M_Cm0 = demo.get_M_Cm0(gray, gray) # camera to object pose

        for i in range(4):
            for j in range(4):
                mat.data[i * 4 + j] = M_Cm0[i][j]

        pub.publish(mat)
        time.sleep(1)

