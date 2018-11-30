#!/usr/bin/env python
import rospy
import camera_demo_arucoTag_kinect as demo
import kinect_subscriber as kinect

import cv2
import numpy as np


if __name__ == '__main__':
    count = demo.START_COUNT
    while 1:
        # get a frame from RGB camera
        # frame_current = get_video()
        frame_current = kinect.get_rgb_image()

        # get a frame from depth sensor
        # depth, depth_raw = get_depth()
        depth, depth_raw = kinect.get_depth()

        # display RGB image
        cv2.imshow('RGB image', frame_current)
        cv2.waitKey(2000)

        # display depth image
        cv2.imshow('Depth image', depth)
        cv2.waitKey(2000)

        # display processed image
        frame_processed = demo.aruco_pose(frame_current)
        cv2.imshow('Processed image', frame_processed)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        # wait for 's' key to save
        elif k == ord('s'):
            save_name = IMG_SAVEPATH + "/pic_" + str(count) + ".png"
            cv2.imwrite(save_name, frame_current)

            save_name = IMG_SAVEPATH + "/pic_" + str(count) + "_d.png"
            cv2.imwrite(save_name, depth)

            save_name = IMG_SAVEPATH + "/pic_" + str(count) + "_raw.png"
            cv2.imwrite(save_name, depth_raw)

            count += 1

    print('here')
