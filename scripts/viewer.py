#!/usr/bin/env python
# coding: UTF-8
import copy

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from ros_posenet.msg import Poses
from sensor_msgs.msg import Image


class Visualize:
    def __init__(self):
        rospy.init_node("visualize")
        rospy.Subscriber("/ros_posenet/result", Poses, self.poses_callback, queue_size=1)
        rospy.Subscriber("/posenet/input", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.color_image = None
        self.poses = None
    
    def poses_callback(self, msg):
        # type:(Poses)->None
        self.poses = msg.poses
    
    def image_callback(self, msg):
        # type:(Image)->None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.color_image = copy.copy(cv_image)
        except CvBridgeError as e:
            print 'Cv_Bridge_Error:', e
        
        if self.poses is None:
            cv2.imshow("window", self.color_image)
            cv2.waitKey(1)
            return
        
        person_id = 0
        for pose in self.poses:
            for key in pose.keypoints:
                color_table = {0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 0, 255), 4: (255, 255, 0)}
                cv2.circle(self.color_image, (int(key.image_position.x), int(key.image_position.y)), 10,
                           (color_table[person_id]),
                           thickness=-1)
                cv2.putText(self.color_image,
                            str((float("{0:.2f}".format(key.position.x)), float("{0:.2f}".format(key.position.y)),
                                 float("{0:.2f}".format(key.position.z)))),
                            (int(key.image_position.x), int(key.image_position.y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1, cv2.LINE_AA)
            person_id += 1
        
        del self.poses[:]
        cv2.imshow("window", self.color_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    Visualize()
    rospy.spin()
