#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


TOPIC_IN = "/autobot01/camera_node/image/compressed"
TOPIC_OUT = "/autobot01/camera_node/image/raw"


class DecoderNode:
    def __init__(self, node_name):
        # Initialize the ROS node
        rospy.init_node(node_name, anonymous=False)

        # parameters
        self.publish_freq = 2.0
        
        # utility objects
        self.bridge = CvBridge()

        # publishers
        self.pub_img = rospy.Publisher(
            # "~image_out",
            TOPIC_OUT,
            Image,
            queue_size=1
        )

        # Storage for the latest image message
        self.latest_image_msg = None

        # subscribers
        self.sub_img = rospy.Subscriber(
            # "~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size=10*1024*1024
            TOPIC_IN, CompressedImage, self.cb_image, queue_size=1, buff_size=10*1024*1024
        )

        # Set up a timer if the publish frequency is specified
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_freq), self.timer_callback)

    def cb_image(self, msg):
        # Store the latest image message
        self.latest_image_msg = msg

    def timer_callback(self, event):
        # Publish the latest image message if there are subscribers
        if self.latest_image_msg and self.pub_img.get_num_connections() > 0:
            self.publish_image(self.latest_image_msg)

    def publish_image(self, msg):
        # Turn 'compressed image message' into 'raw image'
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Turn 'raw image' into 'raw image message'
        out_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        # Maintain original header
        out_msg.header = msg.header
        # Publish image
        self.pub_img.publish(out_msg)


if __name__ == "__main__":
    node = DecoderNode("decoder_node")
    rospy.spin()
