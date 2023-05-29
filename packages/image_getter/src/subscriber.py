#!/usr/bin/env python3

import os
import rospy
from cv_bridge import CvBridge
import cv2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import numpy as np

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        #params:
        self.image = None
        self.br = CvBridge()
        self.num = 0
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        #self.sub = rospy.Subscriber('/'+ os.environ['DUCKIEBOT_NAME'] + "/camera_node/image/raw", Image, self.Imagecallback)
        #self.sub = rospy.Subscriber("/camera/image",Image,self.callback)
        self.sub = rospy.Subscriber('chatter', String, self.callback)

    def callback(self, data):
        img = np.random.randint(255, size=(300, 600, 3))
        rospy.loginfo('Image received...')
        #self.image = self.br.imgmsg_to_cv2(msg)
        if not cv2.imwrite("saved"+str(self.num)+".jpg", img):
            raise Exception("Could not write image")
        cv2.imshow('image',img)
        cv2.waitKey(0)
        self.num += 1

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()

