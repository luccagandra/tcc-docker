#!/usr/bin/env python3

import rospy
import sys
import select
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SendGoalObstacle:
    def __init__(self):
        rospy.init_node('goal_send_planners')

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.update_odom)
        self.send_goal_mavros = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.x, self.y, self.z, self.yaw = input("x y z yaw(rad): ").split()
        self.x, self.y, self.z, self.yaw = float(self.x), float(self.y), float(self.z), float(self.yaw)

        self.x_current = 0
        self.y_current = 0
        self.z_current = 0

        self.rate = rospy.Rate(20)  # 20 Hz

        self.first_iteration = True

        while not rospy.is_shutdown():
            self.publish_setpoint()
            self.check_for_new_setpoint()
            self.rate.sleep()

    def update_odom(self, msg):
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y
        self.z_current = msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.heading = yaw

    def publish_setpoint(self):
        setpoint = PositionTarget()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                             + PositionTarget.IGNORE_YAW_RATE

        setpoint.position.x = self.x
        setpoint.position.y = self.y
        setpoint.position.z = self.z

        if self.first_iteration:
            self.first_iteration = False

            #self.set_mode('OFFBOARD')
            self.arm()

        setpoint.header.stamp = rospy.Time.now()
        self.send_goal_mavros.publish(setpoint)

    def check_for_new_setpoint(self):
        if rospy.is_shutdown():
            return
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                self.x, self.y, self.z, self.yaw = input("x y z yaw(rad): ").split()
                self.x, self.y, self.z, self.yaw = float(self.x), float(self.y), float(self.z), float(self.yaw)
        except Exception as e:
            rospy.logwarn("Failed to get new setpoint: %s", e)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(True)
            rospy.loginfo("Drone armed")
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: %s", e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.set_mode_service(custom_mode=mode)
            rospy.loginfo("Mode set to %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("Setting mode failed: %s", e)

if __name__ == '__main__':
    try:
        SendGoalObstacle()
    except rospy.ROSInterruptException:
        pass
