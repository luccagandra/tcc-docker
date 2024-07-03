#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Vector3
import matplotlib
matplotlib.use('Agg')  # Usar o backend Agg para não precisar de GUI
import matplotlib.pyplot as plt
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ListenContact:
    def __init__(self):
        rospy.init_node('listen2contacts')
        rospy.Subscriber("/rod/contact", ContactsState, self.callback)

        self.bridge = CvBridge()

        self.pub_image = rospy.Publisher('/image_plot', Image, queue_size=10)
        
        self.counter = 0
        self.x_data = []
        self.y_data = []
        self.first_iteration = True

        rospy.spin()

    def callback(self, msg):
        if len(msg.states) > 0:
            force = msg.states[0].total_wrench.force
            torque = msg.states[0].total_wrench.torque
            self.publish_force_and_torque(force, torque)

        if self.first_iteration:
            self.plot_x(0)
            self.first_iteration = False

    def publish_force_and_torque(self, force, torque):
        vector = Vector3()
        vector_t = Vector3()

        vector.x = force.x
        vector.y = force.y
        vector.z = force.z

        vector_t.x = torque.x
        vector_t.y = torque.y
        vector_t.z = torque.z

        self.plot_x(vector.x)

    def plot_x(self, vec_x):
        self.x_data.append(self.counter)
        self.y_data.append(vec_x)

        plt.clf()  # Limpar a figura para evitar sobreposição
        plt.plot(self.x_data, self.y_data, 'o')  # Plotar os dados

        plt.grid(True)  # Adicionar grade
        plt.xlabel('Time')
        plt.ylabel('Force')
        plt.title('Force over collision events')

        plt.savefig('/root/simulator/src/tcc-docker/scripts/images/plot_image.png')  # Salvar como imagem temporária
        plt.close()

        img = cv2.imread('/root/simulator/src/tcc-docker/scripts/images/plot_image.png', cv2.IMREAD_COLOR)

        if len(self.x_data) == 50:
            self.x_data.pop(0)
            self.y_data.pop(0)

        self.counter += 1

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

if __name__ == '__main__':
    loop = ListenContact()
