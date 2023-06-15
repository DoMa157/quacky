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
twist = f'/{CAR}/car_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{CAR}/wheels_driver_node/wheels_cmd_executed'
image = f"/{CAR}/camera_node/image/compressed"


class WheelsDriver(DTROS):
    def __init__(self, node_name):
        super(WheelsDriver, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Subscribers
        self.sub_image = rospy.Subscriber(
            image,
            CompressedImage,
            self.cb_image,
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

        self.cmd_pub = rospy.Publisher(
            twist,
            Twist2DStamped,
            queue_size=1
        )

        self.pub_centers = rospy.Publisher(
            "~centers",
            VehicleCorners,
            queue_size=1
        )
        self.pub_circle_pattern_image = rospy.Publisher(
            "~debug/detection_image/compressed",
            CompressedImage,
            queue_size=1
        )
        self.pub_detection_flag = rospy.Publisher(
            "~detection",
            BoolStamped,
            queue_size=1
        )

        self.twist = Twist2DStamped()
        self.wheel = WheelsCmdStamped()
        self.FSM = Twist2DStamped()

        self.wheel.vel_right = 0.2
        self.wheel.vel_left = 0.2

        self.twist.v = 0.2
        self.twist.omega = 0

        self.bridge = CvBridge()

        self.last_stamp = rospy.Time.now()

        self.log("Initialization completed.")

    def cb_image(self, image_msg):
        vehicle_centers_msg_out = VehicleCorners()
        detection_flag_msg_out = BoolStamped()

        image_cv = self.bridge.compressed_imgmsg_to_cv2(
            image_msg,
            "bgr8"
        )
        (detection, centers) = cv2.findCirclesGrid(
            image_cv,
            patternSize=(7, 3),
        )

        vehicle_centers_msg_out.header = image_msg.header
        vehicle_centers_msg_out.detection.data = detection > 0
        detection_flag_msg_out.header = image_msg.header
        detection_flag_msg_out.data = detection > 0
        if detection > 0:
            if(rospy.Time.now() - self.FSM.header.stamp >= rospy.Duration(3)):
                self.FSM.header.stamp = rospy.Time.now()
                self.FSM.v = 1
                rospy.loginfo("Centers detected. Initiating left turn.")
                # do left turn
                self.twist.omega = 60
        elif (rospy.Time.now() - self.FSM.header.stamp >= rospy.Duration(0.5)):
            if (self.FSM.v == 1):
                self.FSM.header.stamp = rospy.Time.now()
                self.FSM.v = -1
                self.twist.omega = -60
            elif (self.FSM.v == 2):
                self.FSM.header.stamp = rospy.Time.now()
                self.FSM.v = 3
                self.twist.omega = 60
            elif(self.FSM.v == 3):
                self.FSM.header.stamp = rospy.Time.now()
                self.FSM.v = 4
                self.twist.omega = -60
            else:
                self.FSM.header.stamp = rospy.Time.now()
                self.FSM.v = 4
                self.twist.omega = 0
        if(self.FSM.v == -1 and rospy.Time.now() - self.FSM.header.stamp >= rospy.Duration(1)):
            self.FSM.header.stamp = rospy.Time.now()
            self.FSM.v = 2
            self.twist.omega = -60
        # FSM.omega = 0
        # self.pub_FSM.publish(FSM)
        # rospy.sleep(2)
        # FSM.v = -1
        # FSM.omega = -60
        # self.pub_FSM.publish(FSM)
        # rospy.loginfo("Did the left and corrected. Doing right")
        # rospy.sleep(1)
        # FSM.v = 0
        # FSM.omega = 0
        # self.pub_FSM.publish(FSM)
        # rospy.loginfo("Continuing normally.")
        # if the detection is successful add the information about it,
        # otherwise publish a message saying that it was unsuccessful
        if detection > 0:
            points_list = []
            for point in centers:
                center = Point32()
                center.x = point[0, 0]
                center.y = point[0, 1]
                center.z = 0
                points_list.append(center)
            vehicle_centers_msg_out.corners = points_list
            vehicle_centers_msg_out.H = 7
            vehicle_centers_msg_out.W = 3

        self.pub_centers.publish(vehicle_centers_msg_out)
        self.pub_detection_flag.publish(detection_flag_msg_out)

        if self.pub_circle_pattern_image.get_num_connections() > 0:
            cv2.drawChessboardCorners(image_cv, (7, 3), centers, detection)
            image_msg_out = self.bridge.cv2_to_compressed_imgmsg(image_cv)
            self.pub_circle_pattern_image.publish(image_msg_out)
        self.wheels_cmd_pub.publish(self.wheel)
        self.wheels_cmd_executed_pub.publish(self.wheel)
        self.cmd_pub.publish(self.twist)

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
            rospy.spin()
        except KeyboardInterrupt:
            rospy.on_shutdown(runner.on_shutdown)
