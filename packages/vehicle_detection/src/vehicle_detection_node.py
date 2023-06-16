#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, VehicleCorners, WheelsCmdStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage

vehicle = os.environ['VEHICLE_NAME']
wheels_cmd = f'/{vehicle}/wheels_driver_node/wheels_cmd'
cmd = f'/{vehicle}/vehicle_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{vehicle}/wheels_driver_node/wheels_cmd_executed'
image = f"/{vehicle}/camera_node/image/compressed"

class VehicleDetectionNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(VehicleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Subscribers
        self.sub_image = rospy.Subscriber(
            image,
            CompressedImage,
            self.cb_image,
            queue_size=1
        )
        
        # Publishers
        # self.pub_centers = rospy.Publisher(
        #     "~centers",
        #     VehicleCorners,
        #     queue_size=1
        # )
        # self.pub_circle_pattern_image = rospy.Publisher(
        #     "~debug/detection_image/compressed",
        #     CompressedImage,
        #     queue_size=1
        # )
        self.pub_detection_flag = rospy.Publisher(
            "~detection",
            BoolStamped,
            queue_size=1
        )

        self.name = node_name
        
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()
    
        rospy.loginfo(f"[{node_name}] Initialization completed.")

    def cb_image(self, image_msg):
        # vehicle_centers_msg_out = VehicleCorners()
        detection_flag_msg_out = BoolStamped()

        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        (detection, centers) = cv2.findCirclesGrid(image_cv, patternSize=(7, 3))

        # vehicle_centers_msg_out.header = image_msg.header
        # vehicle_centers_msg_out.detection.data = detection > 0
        detection_flag_msg_out.header = image_msg.header
        detection_flag_msg_out.data = detection > 0

        # if detection > 0:
        #     points_list = []
        #     for point in centers:
        #         center = Point32()
        #         center.x = point[0, 0]
        #         center.y = point[0, 1]
        #         center.z = 0
        #         points_list.append(center)
        #     vehicle_centers_msg_out.corners = points_list
        #     vehicle_centers_msg_out.H = 7
        #     vehicle_centers_msg_out.W = 3

        # self.pub_centers.publish(vehicle_centers_msg_out)
        self.pub_detection_flag.publish(detection_flag_msg_out)

        # if self.pub_circle_pattern_image.get_num_connections() > 0:
        #     cv2.drawChessboardCorners(image_cv, (7, 3), centers, detection)
        #     image_msg_out = self.bridge.cv2_to_compressed_imgmsg(image_cv)
        #     self.pub_circle_pattern_image.publish(image_msg_out)
    
    def on_shutdown(self):
        rospy.loginfo(f'[{self.name}] Shutting down...')
        
if __name__ == "__main__":
    node = VehicleDetectionNode("vehicle_detection_node")
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.on_shutdown(node.on_shutdown)
