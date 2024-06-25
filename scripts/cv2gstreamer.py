#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class GStreamerConverter:
    def __init__(self):
        rospy.init_node('img_to_qground')

        # ROS

        self.bridge = CvBridge()

        front_topic = "/camera_front/color/image_raw"

        self.which_image = 'front'

        rospy.Subscriber(front_topic, Image, self.callback)

        # Define the gstreamer sink
        #self.gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000000 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        #self.gst_str_rtp = "appsrc ! videoconvert ! x264enc bitrate=1000000 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        #self.gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=10000 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        self.gst_str_rtp = "appsrc ! videoconvert ! x264enc ! rtph264pay ! udpsink host=127.0.0.1 port=5600"

        rospy.spin()

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # GStreamer

        # Cam properties
        fps = 30.
        frame_width = 640
        frame_height = 480

        self.out = cv2.VideoWriter(self.gst_str_rtp, 0, 30, (frame_width, frame_height), True)

        img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)

        self.out.write(image)

if __name__ == '__main__':
    loop = GStreamerConverter()
