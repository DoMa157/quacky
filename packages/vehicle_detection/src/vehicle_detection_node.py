#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import BoolStamped, VehicleCorners, Twist2DStamped, WheelsCmdStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage

CAR = os.environ['VEHICLE_NAME']
wheels_cmd = f'/{CAR}/wheels_driver_node/wheels_cmd'
cmd = f'/{CAR}/car_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{CAR}/wheels_driver_node/wheels_cmd_executed'

class VehicleDetectionNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(VehicleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        veh_name = os.environ['VEHICLE_NAME']


if __name__ == "__main__":
    vehicle_detection_node = VehicleDetectionNode("vehicle_detection")
    rospy.spin()
