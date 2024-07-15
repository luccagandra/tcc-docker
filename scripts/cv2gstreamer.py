#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class GStreamerConverter:
    def __init__(self):
        rospy.init_node('img_to_qground')

        # ROS

        self.bridge = CvBridge()

        front_topic = "/camera_front/color/image_raw"
        plot_topic = "/image_plot"

        rospy.Subscriber(front_topic, Image, self.callback_rgb)
        rospy.Subscriber(plot_topic, Image, self.callback_plot)

        self.rgb_image = None
        self.plot_image = None

        self.matplotlib = rospy.get_param('/cv2gstreamer/matplotlib')

        # Define the gstreamer sink
        #self.gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=1000000 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        #self.gst_str_rtp = "appsrc ! videoconvert ! x264enc bitrate=1000000 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        #self.gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=10000 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        self.gst_str_rtp = "appsrc ! videoconvert ! x264enc ! rtph264pay ! udpsink host=127.0.0.1 port=5600"

        rospy.spin()

    def callback_rgb(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if self.plot_image is not None and self.matplotlib == True:
            self.gstreamer()

        if self.matplotlib == False:
            self.gstreamer()

    def callback_plot(self, msg):
        try:
            self.plot_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.rgb_image is not None:
            self.gstreamer()
    
    def gstreamer(self):
        
        if self.matplotlib:
            img_out = np.vstack((self.rgb_image, self.plot_image))

            cv2.resize(img_out, (0,0), fx=0.5, fy=0.5) 
            
            # GStreamer
            # Cam properties
            frame_width = 640
            frame_height = 960

            # Pipe
            self.out = cv2.VideoWriter(self.gst_str_rtp, 0, 30, (frame_width, frame_height), True)
            self.out.write(img_out)
        else:
            img_out = self.rgb_image

            # Cam properties
            frame_width = 640
            frame_height = 480

            self.out = cv2.VideoWriter(self.gst_str_rtp, 0, 30, (frame_width, frame_height), True)
            self.out.write(img_out)

if __name__ == '__main__':
    loop = GStreamerConverter()
