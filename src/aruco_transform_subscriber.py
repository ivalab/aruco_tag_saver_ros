#!/usr/bin/env python
import rospy
import std_msgs

import aruco_transform as publisher

TOPIC_NAME = 'aruco_transform'

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print data
    
def listener():
    rospy.init_node('aruco_transform_listener', anonymous=True)
    rospy.Subscriber(publisher.TOPIC_NAME, std_msgs.msg.Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

