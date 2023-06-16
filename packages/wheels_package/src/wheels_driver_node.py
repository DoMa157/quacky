#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

vehicle = os.environ['VEHICLE_NAME']
wheels_cmd = f'/{vehicle}/wheels_driver_node/wheels_cmd'
wheels_cmd_executed = f'/{vehicle}/wheels_driver_node/wheels_cmd_executed'
vehicle_detected = f'/{vehicle}/vehicle_detection_node/detection'


class WheelsDriverNode(DTROS):
    def __init__(self, node_name):
        super(WheelsDriverNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Subscribers
        self.vehicle_detected_sub = rospy.Subscriber(
            vehicle_detected,
            BoolStamped,
            self.cb_wheels,
            queue_size=1
        )

        # Publishers
        self.wheels_cmd_pub = rospy.Publisher(
            wheels_cmd,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_executed_pub = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.name = node_name

        self.wheel = WheelsCmdStamped()

        self.wheel.vel_right = 0.1
        self.wheel.vel_left = 0.1

        rospy.loginfo(f"[{node_name}] Initialization completed.")

        def cb_wheels(self, msg):
            if msg.data:
                self.wheel.vel_right = 0.0
                self.wheel.vel_left = 0.0
            else:
                self.wheel.vel_right = 0.1
                self.wheel.vel_left = 0.1

            self.wheels_cmd_pub.publish(self.wheel)
            self.wheels_cmd_executed_pub.publish(self.wheel)

    def on_shutdown(self):
        rospy.loginfo(f'[{self.name}] Shutting down...')
        wheel = WheelsCmdStamped()
        wheel.vel_right = 0
        wheel.vel_left = 0
        self.wheels_cmd_pub.publish(wheel)


if __name__ == "__main__":
    node = WheelsDriverNode("wheels_driver_node")
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.on_shutdown(node.on_shutdown)
