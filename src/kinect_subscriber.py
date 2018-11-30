#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import numpy as np
from PIL import Image as PILImage
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = "/camera/rgb/image_color"
DEPTH_TOPIC = "/camera/depth/image_raw"


def get_rgd_image(show=False):
    #print("CALLING GET_KINECT_IMAGE")
    rospy.init_node("kinect_subscriber")
    rgb = rospy.wait_for_message(IMAGE_TOPIC, Image)
    depth = rospy.wait_for_message(DEPTH_TOPIC, Image)

    # Convert sensor_msgs.Image readings into readable format
    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb, rgb.encoding)
    depth = bridge.imgmsg_to_cv2(depth, depth.encoding)

    image = rgb
    image[:, :, 1] = depth
    if (show):
        im = PILImage.fromarray(image, 'RGB')
        im.show()

    return image


def get_rgb_image(show=False):
    rospy.init_node("kinect_subscriber")
    rgb = rospy.wait_for_message(IMAGE_TOPIC, Image)

    # Convert sensor_msgs.Image readings into readable format
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(rgb, rgb.encoding)

    if (show):
        im = PILImage.fromarray(image, 'RGB')
        im.show()

    return image


def get_depth():
    rospy.init_node("kinect_subscriber")
    depth_raw = rospy.wait_for_message(DEPTH_TOPIC, Image)

    # Convert sensor_msgs.Image readings into readable format
    bridge = CvBridge()
    depth_raw = bridge.imgmsg_to_cv2(depth_raw, depth_raw.encoding)
    depth = depth_raw.astype(np.uint8)

    return depth, depth_raw


if __name__ == '__main__':
    image = get_image(show=True)

