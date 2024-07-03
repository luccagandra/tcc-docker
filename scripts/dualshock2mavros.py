#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int32, Float64
from mavros_msgs.srv import CommandTOL
import math
from geometry_msgs.msg import Twist, PoseStamped
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
        
        rospy.Subscriber("/icts_uav/odometry", Odometry, self.update_odom)

        self.velocity = 2
        self.velocity_angular = 0.03
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

        self.x = but[3]
        self.o = but[2]
        self.tri = but[1]
        self.square = but[0]
        self.direc_left = but[14]
        self.direc_right = but[16]
        self.direc_up = but[15]
        self.direc_down = but[17]
        self.l3 = but[12]
        self.r3 = but[13]
        self.options = but[9]
        self.share = but[8]
        self.l1 = but[4]
        self.r1 = but[6]
        self.touchpad_button = but[11]
        self.ps = but[10]
        self.l2_total = but[5]
        self.r2_total = but[7]

        self.l_hor = axes[0]
        self.l_vert = axes[1]
        self.r_vert = axes[3]
        self.r_hor = -axes[2]
        self.l2 = axes[4]
        self.r2 = axes[5]

        self.main()

    #def callback2(self, msg_2):
    #    self.heading = msg_2.data

    def main(self):

        if self.o == 1:
            is_landing = self.land_client()
            rospy.loginfo("land sent %d", is_landing.success)

        z_passed = False

        if self.r2_total == 1:
            self.message.linear.z = 0.75*self.velocity * self.r2
            z_passed = True

        if self.l2_total == 1:
            self.message.linear.z = -0.75*self.velocity * self.l2
            z_passed = True

        if not z_passed == True:
            self.message.linear.z = 0

        if self.l_hor > 0.1 or self.l_hor < -0.1:
            self.message.linear.y = self.velocity * self.l_hor
        else:
            self.message.linear.y = 0

        if self.l_vert > 0.1 or self.l_vert < -0.1:
            self.message.linear.x = self.velocity * self.l_vert
        else:
            self.message.linear.x = 0

        if self.r_hor > 0.1:
            self.message.angular.z = -self.velocity_angular * abs(self.r_hor)

        elif self.r_hor < -0.1:
            self.message.angular.z = self.velocity_angular * abs(self.r_hor)
        else:
            self.message.angular.z = 0

        if self.x == 1: 
            os.system("rosrun mavros mavsafety arm")
            rospy.sleep(1)
            os.system("rosrun mavros mavsys mode -c OFFBOARD")


        if self.tri == 1 and self.execution == 1:
            self.estimate_tower()
            self.execution += 1         
            rospy.sleep(2)

        if self.tri == 1 and self.execution == 0:
            self.save_x1()
            self.execution += 1
            print("x1 y1 yaw1", self.x1, self.y1, self.yaw1)
            rospy.sleep(2)

        #    os.system("rosrun mavros mavcmd takeoff 0 0 0 0 2")

        #if self.tri == 1:
        #    os.system("rosservice call /mavros/set_stream_rate 0 10 1")
        #    os.system("rosrun mavros mavsys mode -c OFFBOARD")

    def save_x1(self):
        self.x1 = self.x_curr
        self.y1 = self.y_curr
        self.yaw1 = self.heading

    def estimate_tower(self):
        
        x1 = self.x1
        y1 = self.y1
        yaw1 = math.tan(self.yaw1)

        x2 = self.x_curr
        y2 = self.y_curr
        yaw2 = math.tan(self.heading)

        print("x1 y1 yaw1", x1, y1, yaw1)
        print("x2 y2 yaw2", x2, y2, yaw2)

        b1 = y1 - yaw1*x1
        b2 = y2 - yaw2*x2

        xt = (b1 - b2)/(yaw2 - yaw1)
        yt = yaw2*xt + b2
        yt1 = yaw1*xt + b1

        print("Posição aproximada da torre: ", xt, yt1)

    def publish_message(self, event):
        self.pub.publish(self.message)

if __name__ == '__main__':
    loop = Dualshock()