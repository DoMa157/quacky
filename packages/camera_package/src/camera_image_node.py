#!/usr/bin/env python3

import cv2
import rospy
import os
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

TOPIC_NAME = f'/{os.environ["VEHICLE_NAME"]}/camera_node/image/compressed'

class ImageDetection(DTROS):
    def __init__(self, node_name):
        super(ImageDetection, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
            TOPIC_NAME,
            CompressedImage,
            self.callback,
            queue_size=1
        )

        self.pub = rospy.Publisher(
            'camera_view',
            CompressedImage, 
            queue_size=1
        )

    def callback(self, msg):
        print(f'Received: ${type(msg)}')
        self.pub.publish(msg)

if __name__ == '__main__':
    node = ImageDetection(node_name='image_view_node')
    rospy.spin()
    