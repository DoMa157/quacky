#!/usr/bin/env python3

import cv2
import rospy
import os
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

# actual topic name where the camera output is stored.
# Replace 'bot_name' with duckiebots' name
TOPIC_NAME = f'/{os.environ["VEHICLE_NAME"]}/camera_node/image/compressed'

class MyNode(DTROS):
    def __init__(self, node_name):
        super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # bridge for converting from/to CompressedImage to/from cv2
        self.bridge = CvBridge()

        # subscriber to the camera output topic. Constructor arguments are following:

        # TOPIC_NAME - topic name to which subscribe, if topic with name TOPIC_NAME
        # doesn't exist, new topic will be created

        # CompressedImage - messaging object type, e.g, in the developer manual example
        # String was used to publish/subscribe for text messages

        # self.callback - callback function which will be invoked on each notify,
        # method self.callback implemented below
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1)

        # publisher to upload filtered images. Constructor arguments are following:

        # 'filtered_images' - name of the new topic
        # CompressedImage - type of the published object
        self.pub = rospy.Publisher('filtered_images', CompressedImage, queue_size=1)

def callback(self, msg):
    print(f'received message with type ${type(msg)}') # ideally, <class 'CompressedImage'>

    # converting from CompressedImage to cv2. yeah... that's actual name of the method 
    converted_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    # applying the filter, here, canny filter with thresholds 50 and 150
    filtered_img = cv2.Canny(converted_img, 50, 150)

    # converting back to CompressedImage
    compressed_result_img = self.bridge.cv2_to_compressed_imgmsg(filtered_img)

    # publishing the result
    self.pub.publish(compressed_result_img)

if __name__ == '__main__':
    node = MyNode(node_name='my_node') # instantiation
    rospy.spin() # blocks until rospy.is_shutdown()ls
    