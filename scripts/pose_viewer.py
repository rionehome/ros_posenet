#!/usr/bin/env python
# coding: UTF-8
import rospy
from ros_posenet.msg import Poses
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt


class Visualize:
    def __init__(self):
        rospy.Subscriber("/ros_posenet/result", Poses, self.poses_callback)
        rospy.Subscriber("/posenet/input", Image, self.image_callback)
    
    def poses_callback(self):
        pass
    
    def image_callback(self):
        pass


if __name__ == '__main__':
    Visualize()
    rospy.spin()
