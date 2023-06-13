#!/usr/bin/env python3

import os
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown.dtros import DTROS, NodeType

CAR = os.environ['VEHICLE_NAME']
wheels_cmd = f'/{CAR}/wheels_driver_node/wheels_cmd'
wheels_cmd_executed = f'/{CAR}/wheels_driver_node/wheels_cmd_executed'


class WheelsDriver(DTROS):
    def __init__(self, node_name):
        super(WheelsDriver, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.wheels_cmd_pub = rospy.Publisher(
            wheels_cmd,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_sub = rospy.Subscriber(
            wheels_cmd,
            WheelsCmdStamped,
            self.wheels_cmd_cb,
            queue_size=1
        )

        self.wheels_cmd_executed_pub = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_executed_sub = rospy.Subscriber(
            wheels_cmd_executed,
            WheelsCmdStamped,
            self.wheels_cmd_executed_cb,
            queue_size=1
        )

    def wheels_cmd_cb(self, wheels):
        rospy.loginfo(f'Command: {type(wheels)}')

    def wheels_cmd_executed_cb(self, wheels):
        rospy.loginfo(f'Command: {type(wheels)}')

    def run(self):
        wheel = WheelsCmdStamped()
        wheel.vel_right = 0.2
        wheel.vel_left = 0.2

        # twist = Twist2DStamped()
        # twist.v = 40
        # twist.omega = 10
        # This is for turning

        while not rospy.is_shutdown():
            self.wheels_cmd_pub.publish(wheel)
            self.wheels_cmd_executed_pub.publish(wheel)
            # self.cmd_pub.publish(twist)
    
    def on_shutdown(self):
        rospy.loginfo('Shutting down...')
        wheel = WheelsCmdStamped()
        wheel.vel_right = 0
        wheel.vel_left = 0
        self.wheels_cmd_pub.publish(wheel)

if __name__ == "__main__":
    runner = WheelsDriver("wheels_driver_node")
    while not rospy.is_shutdown():
        try:
            runner.run()
            rospy.spin()
        except:
            rospy.on_shutdown(runner.on_shutdown)
