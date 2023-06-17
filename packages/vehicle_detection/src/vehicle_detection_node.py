#!/usr/bin/env python3

import cv2 as cv
import rospy
import os
import numpy as np
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


HOST = os.environ['VEHICLE_NAME']
TOPIC_NAME = f'/{HOST}/camera_node/image/compressed'
class VehicleDetectionNode(DTROS):
    def __init__(self, node_name):
        
        super(VehicleDetectionNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        # for converting from CompressedImage to cv2 and vice versa
        self.bridge = CvBridge()

        # subscribing to topic TOPIC_NAME, messaging object type is CompressedImage, on each notify callback is called
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1, buff_size="10MB")
        # publishing to the new topic 'image_pub', messaging object type is CompressedImage
        #  self.pub = rospy.Publisher('image_pub', CompressedImage, queue_size=1)
        self.remote_pub = rospy.Publisher('duckiebot_detected', Bool,queue_size=1)

    def callback(self, msg):
        # print(f'callback with type ${type(msg)}')
        # converting CompressedImage to cv2
        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # vehicle mask
        detected,patterns = cv.findCirclesGrid(img_cv2, (7, 3)) 
        # cv.drawChessboardCorners(img_cv2,(7,3), patterns, True)
            # converting filtered result to CompressedImage
        # img_filtered_compressed = self.bridge.cv2_to_compressed_imgmsg(img_cv2)
            # publishing to 'image_pub'
        # self.pub.publish(img_filtered_compressed)
        self.remote_pub.publish(detected)


if __name__ == '__main__':
    node = VehicleDetectionNode(node_name='vehicle_detection_node')
    rospy.spin()