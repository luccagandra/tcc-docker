#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int32, Float64
from mavros_msgs.srv import CommandTOL
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import os

class Dualshock:
    def __init__(self):
        rospy.init_node('dualshock_to_mavros')

        self.pub = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

        rospy.Subscriber('/game_control/joy', Joy, self.callback, tcp_nodelay=False)
        #rospy.Subscriber("/mavros/global_position/compass_hdg",
        #                 Float64, self.callback2)

        self.land_client = rospy.ServiceProxy(
            "/mavros/cmd/land", CommandTOL)
        
        rospy.Subscriber("/mavros/global_position/local", Odometry, self.update_odom)

        self.velocity = 2
        self.velocity_angular = 0.5
        self.heading = 0.0
        self.message = Twist()
        self.execution = 0
        self.x1 = 0
        self.y2 = 0
        self.yaw1 = 0

        self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_message)
        
        rospy.spin()

    def update_odom(self, msg):
        self.x_curr = msg.pose.pose.position.x
        self.y_curr = msg.pose.pose.position.y
        self.z_curr = msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation

        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.heading = yaw

    def callback(self, msg):
        axes = msg.axes
        but = msg.buttons

        self.x = but[1]
        self.o = but[2]
        self.tri = but[3]
        self.square = but[0]
        self.l1 = but[4]
        self.r1 = but[5]
        self.l2_total = but[6]
        self.r2_total = but[7]

        self.l_hor = axes[0]
        self.l_vert = axes[1]
        self.r_vert = axes[3]
        self.r_hor = -axes[2]
        self.R2 = axes[5]

        self.main()

    #def callback2(self, msg_2):
    #    self.heading = msg_2.data

    def main(self):

        if self.o == 1:
            is_landing = self.land_client()
            rospy.loginfo("land sent %d", is_landing.success)

        z_passed = False

        if self.R2 > 0.1:
            self.message.linear.z = 0.75*self.velocity
            z_passed = True

        if self.R2 < -0.1:
            self.message.linear.z = -0.75*self.velocity 
            z_passed = True

        if not z_passed == True:
            self.message.linear.z = 0

        # Roll -> Movimento para esquerda e direita (linear.y)
        if self.r_hor > 0.1 or self.r_hor < -0.1:
            self.message.linear.y = self.velocity * self.l_hor
        else:
            self.message.linear.y = 0

        # Pitch -> Movimento para frente e trás (linear.x)
        if self.l_vert > 0.1 or self.l_vert < -0.1:
            self.message.linear.x = self.velocity * self.l_vert
        else:
            self.message.linear.x = 0

        # Movimento angular L1 e R1
        if self.l1:
            self.message.angular.z = -self.velocity_angular
        elif self.r1:
            self.message.angular.z = self.velocity_angular
        else:
            self.message.angular.z = 0

        if self.x == 1: 
            os.system("rosrun mavros mavsafety arm")
            rospy.sleep(1)
            os.system("rosrun mavros mavsys mode -c OFFBOARD")

        #    os.system("rosrun mavros mavcmd takeoff 0 0 0 0 2")

        #if self.tri == 1:
        #    os.system("rosservice call /mavros/set_stream_rate 0 10 1")
        #    os.system("rosrun mavros mavsys mode -c OFFBOARD")

    def publish_message(self, event):
        self.pub.publish(self.message)

if __name__ == '__main__':
    loop = Dualshock()