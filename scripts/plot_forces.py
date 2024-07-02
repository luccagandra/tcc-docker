#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt

class ListenContact:
    def __init__(self):

        rospy.init_node('listen2contacts')

        rospy.Subscriber("/rod/contact", ContactsState, self.callback)

        self.pub = rospy.Publisher('/rod/force_vector', Vector3, queue_size=100)
        self.pub2 = rospy.Publisher('/rod/torque_vector', Vector3, queue_size=100)

        plt.ion()
        plt.show()
        rospy.spin()

    def callback(self, msg):

        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        
        if len(msg.states) > 0:
            force = msg.states[0].total_wrench.force
            torque = msg.states[0].total_wrench.torque

            self.plot(force)
            print("1")
            self.publish_force_and_torque(force, torque)
        else:
            pass

    def plot(self, force):
        plt.plot(force[0], '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    def publish_force_and_torque(self, force, torque):
        
        vector = Vector3()
        vector_t = Vector3()

        vector.x = force.x
        vector.y = force.y
        vector.z = force.z

        vector_t.x = torque.x
        vector_t.y = torque.y
        vector_t.z = torque.z

        self.pub.publish(vector)
        self.pub2.publish(vector_t)

if __name__ == '__main__':
    loop = ListenContact()
