#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
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
            queue_size=2
        )

    def get_steer_matrix_left_lane_markings(self, shape) -> np.ndarray:
        # TODO: implement your own solution here
        steer_matrix_left = np.zeros(shape)
        steer_matrix_left[:, shape[1]//3:] = -1
        # ---
        return steer_matrix_left


    def get_steer_matrix_right_lane_markings(self, shape) -> np.ndarray:
        steer_matrix_right = np.zeros(shape)
        steer_matrix_right[:, :shape[1]*2//3] = 1
        # ---
        return steer_matrix_right


    def detect_lane_markings(self, image: np.ndarray):
        
        sens = 125
        white_lower_hsv = np.array([0, 0, 255-sens])
        white_upper_hsv = np.array([255, sens, 255])
        yellow_lower_hsv = np.array([10, 50, 80])
        yellow_upper_hsv = np.array([30, 255, 255])

        imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        imggray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        sigma = np.pi

        img_gaussian = cv2.GaussianBlur(imggray,(0,0), sigma)
        sobelx = cv2.Sobel(img_gaussian, cv2.CV_64F, 1, 0)
        sobely = cv2.Sobel(img_gaussian, cv2.CV_64F, 0, 1)
        Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)    

        thresh_w = 10
        thresh_y = 20

        mask_mag_white = (Gmag > thresh_w)
        mask_mag_yellow = (Gmag > thresh_y)
        
        mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
        mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

        width = image.shape[1]
        mask_left = np.ones(sobelx.shape)
        mask_left[:,int(np.floor(width/2)):width + 1] = 0
        mask_right = np.ones(sobelx.shape)
        mask_right[:,0:int(np.floor(width/2))] = 0
        mask_sobelx_pos = (sobelx > 0)
        mask_sobelx_neg = (sobelx < 0)
        mask_sobely_pos = (sobely > 0)
        mask_sobely_neg = (sobely < 0)

        mask_left_edge = mask_left * mask_mag_yellow * mask_sobelx_neg * mask_sobely_neg * mask_yellow
        mask_right_edge = mask_right * mask_mag_white * mask_sobelx_pos * mask_sobely_neg * mask_white

        mask_left_edge = mask_left * mask_mag_yellow * mask_sobelx_neg * mask_sobely_neg * mask_yellow
        mask_right_edge = mask_right * mask_mag_white * mask_sobelx_pos * mask_sobely_neg * mask_white

        # left is yellow, right is white
        return mask_left_edge, mask_right_edge

    def callback(self, msg):
        print(f'Received: ${type(msg)}')

        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        left, right = self.detect_lane_markings(image)

        l, r = self.bridge.cv2_to_compressed_imgmsg(left), self.bridge.cv2_to_compressed_imgmsg(right)

        self.pub.publish(l, r)

if __name__ == '__main__':
    node = ImageDetection(node_name='left_right_markings_node')
    rospy.spin()
