#!/usr/bin/env python
import rospy
import camera_demo_arucoTag_kinect as demo
import kinect_subscriber as kinect

import cv2
import numpy as np
import time

from PIL import Image as PILImage


if __name__ == '__main__':
    # reset the camera tilt
#    ctx = freenect.init()
#    dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
#    freenect.set_tilt_degs(dev, TILT)
    # python wrapper dont register for video/depth so pointer needs to be shutdown before inquiring video
#    freenect.shutdown(ctx)

    count = demo.START_COUNT
#    while 1:
    while count < 1:
        # get a frame from RGB camera
        # frame_current = get_video()
        frame_current = kinect.get_rgb_image()

        # get a frame from depth sensor
        # depth, depth_raw = get_depth()
        depth, depth_raw = kinect.get_depth()

        # display RGB image
        cv2.imshow('RGB image', frame_current)
        cv2.waitKey(1000)

        # display depth image
        cv2.imshow('Depth image', depth)
        cv2.waitKey(4000)
        cv2.destroyAllWindows()

        count += 1

#        frame_processed = demo.aruco_pose(frame_current)
        # display processed image
#        cv2.imshow('Processed image', frame_processed)
#
#
#        # quit program when 'esc' key is pressed
#        k = cv2.waitKey(5) & 0xFF
#        if k == 27:
#            break
#        # wait for 's' key to save
#        elif k == ord('s'):
#            save_name = IMG_SAVEPATH + "/pic_" + str(count) + ".png"
#            cv2.imwrite(save_name, frame_current)
#
#            save_name = IMG_SAVEPATH + "/pic_" + str(count) + "_d.png"
#            cv2.imwrite(save_name, depth)
#
#            save_name = IMG_SAVEPATH + "/pic_" + str(count) + "_raw.png"
#            cv2.imwrite(save_name, depth_raw)
#
#            count += 1
#
#        elif k == ord('t'):
#            # conflit registeration between video and tilt, annoying
#            freenect.sync_stop()
#            time.sleep(0.1)
#
#            ctx = freenect.init()
#            dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
#
#            TILT = min(TILT + TILT_STEP, TILT_MAX)
#            print "Setting TILT: ", TILT
#            freenect.set_tilt_degs(dev, TILT)
#
#            freenect.shutdown(ctx)
#            time.sleep(0.1)
#
#
#        elif k == ord('g'):
#            # conflit registeration between video and tilt, annoying
#            freenect.sync_stop()
#            time.sleep(0.1)
#
#            ctx = freenect.init()
#            dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
#
#            TILT = max(TILT - TILT_STEP, TILT_MIN)
#            print "Setting TILT: ", TILT
#            freenect.set_tilt_degs(dev, TILT)
#
#            freenect.shutdown(ctx)
#            time.sleep(0.1)
#
#    cv2.destroyAllWindows()
