#!/usr/bin/env python3

import os
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from duckietown.dtros import DTROS, NodeType, TopicType
from sensor_msgs.msg import Joy

class WheelsDriver(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(WheelsDriver, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # emergency stop disabled by default
        self.e_stop = False

        # Add the node parameters to the parameters dictionary
        self._speed_gain = self.setup_param("~speed_gain", 0.0)
        self._steer_gain = self.setup_param("~steer_gain", 0.0)
        self._bicycle_kinematics = self.setup_param("~bicycle_kinematics", False)
        self._simulated_vehicle_length = self.setup_param("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_joy_override = rospy.Publisher(
            "~joystick_override", BoolStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_e_stop = rospy.Publisher(
            "~emergency_stop", BoolStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Subscription to the joystick command
        self.sub_joy = rospy.Subscriber("~joy", Joy, self.joy_cb, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.estop_cb, queue_size=1)
    
    def estop_cb(self, estop_msg):
        """
        Callback that process the received :obj:`BoolStamped` messages.

        Args:
            estop_msg (:obj:`BoolStamped`): the emergency_stop message to process.
        """
        self.e_stop = estop_msg.data

    def joy_cb(self, joy_msg):
        """
        Callback that process the received :obj:`Joy` messages.

        Args:
            joy_msg (:obj:`Joy`): the joystick message to process.
        """
        # Navigation buttons
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        # Left stick V-axis. Up is positive
        car_cmd_msg.v = joy_msg.axes[1] * self._speed_gain
        if self._bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = joy_msg.axes[3] * self._steer_gain
            car_cmd_msg.omega = car_cmd_msg.v / self._simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = joy_msg.axes[3] * self._steer_gain
        self.pub_car_cmd.publish(car_cmd_msg)

        # Back button: Stop LF
        if joy_msg.buttons[6] == 1:
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = True
            self.log("override_msg = True")
            self.pub_joy_override.publish(override_msg)

        # Start button: Start LF
        elif joy_msg.buttons[7] == 1:
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = False
            self.log("override_msg = False")
            self.pub_joy_override.publish(override_msg)

        # Y button: Emergency Stop
        elif joy_msg.buttons[3] == 1:
            self.e_stop = not self.e_stop
            estop_msg = BoolStamped()
            estop_msg.header.stamp = joy_msg.header.stamp
            estop_msg.data = self.e_stop
            self.pub_e_stop.publish(estop_msg)

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                self.logwarn("No binding for joy_msg.buttons = %s" % str(joy_msg.buttons))

        print(joy_msg.axes)
    
    def on_shutdown(self):
        """
        Shutdown procedure.
        Publishes a zero velocity command at shutdown.
        """
        rospy.loginfo("Shutting down node %s" % self.node_name)
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.v = 0.0
        car_cmd_msg.omega = 0.0
        self.pub_car_cmd.publish(car_cmd_msg)
    
    def setup_param(self,param_name,default_value):
        try:
            value = rospy.get_param(param_name, default_value)
        except KeyError:
            rospy.logwarn("[%s] Parameter %s not found, defaulting to %s" %(self.node_name,param_name,default_value))
            value = default_value
        rospy.set_param(param_name,value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == "__main__":
    runner = WheelsDriver("joy_mapper")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        runner.on_shutdown()
        rospy.loginfo("Shutting down joy_mapper")