#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class SendGoalObstacle:
    def __init__(self):

        rospy.init_node('goal_send_planners')

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.update_odom)

        self.send_goal_mavros = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, latch=True)

        self.quat_desired = quaternion_from_euler (0, 0, 3.14159)

        self.x, self.y, self.z, self.yaw = input("x y z yaw(rad): ").split()
        self.callback()

        self.x_current = 0
        self.y_current = 0
        self.z_current = 0

        rospy.spin()

    def update_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation

        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.heading = yaw

        #print(self.x, self.y, self.z, self.heading)

    def callback(self):

        send_goal = PoseStamped()

        send_goal.pose.position.x = float(self.x) # -y
        send_goal.pose.position.y = float(self.y)  # x
        send_goal.pose.position.z = float(self.z)   

        quaternion = quaternion_from_euler(0,0,float(self.yaw)) # Roll, Pitch e Yaw

        send_goal.pose.orientation.x = quaternion[0]
        send_goal.pose.orientation.y = quaternion[1]
        send_goal.pose.orientation.z = quaternion[2]
        send_goal.pose.orientation.w = quaternion[3]

        #send_goal.header.seq = "1"
        #send_goal.header.stamp = ""
        send_goal.header.frame_id = 'world' 

        self.send_goal_mavros.publish(send_goal)

if __name__ == '__main__':
    loop = SendGoalObstacle()

