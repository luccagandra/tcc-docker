#!/usr/bin/env python3

import rospy
import sys
import select
import threading
from std_msgs.msg import Float32
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SendGoalObstacle:
    def __init__(self):
        rospy.init_node('goal_send_planners')

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.update_odom)
        rospy.Subscriber("/rod/force_vector", Vector3, self.update_force)

        self.send_goal_mavros = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.send_setpoint = rospy.Publisher('/rod/control/setpoint', Float32, queue_size=10)
        self.send_control = rospy.Publisher('/rod/control/control_output', Float32, queue_size=10)
        self.send_reference = rospy.Publisher('/rod/control/reference', Float32, queue_size=10)

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.x, self.y, self.z = input("x y z: ").split()
        self.x, self.y, self.z = float(self.x), float(self.y), float(self.z)

        self.x_current = 0
        self.y_current = 0
        self.z_current = 0

        self.rate = rospy.Rate(20)  # 20 Hz

        self.iteration = 0

        self.control_mode = "position"

        """
        PID
        """

        self.kp = 0.15
        self.ki = 0.0
        self.kd = 0.05

        self.force_setpoint = 0.0

        self.force_x = 0.0
        self.control_output = 0.0

        self.integral = 0
        self.prev_error = 0

        # Thread for user input
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        while not rospy.is_shutdown():
            self.publish_setpoint()
            self.compute_control()
            self.rate.sleep()

    def update_odom(self, msg):
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y
        self.z_current = msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.heading = yaw

    def update_force(self, msg):
        self.force_x = -msg.x

    def compute_control(self):
        stp = Float32()
        ctrl = Float32()
        reference = Float32()

        if self.control_mode == "velocity":
            error = self.force_setpoint - self.force_x
            self.integral += error
            derivative = error - self.prev_error

            self.control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            
            stp.data = self.force_setpoint
            ctrl.data = self.control_output
            reference = self.force_x
        else:
            stp.data = 0
            ctrl.data = 0
            reference = 0

        self.send_setpoint.publish(stp)
        self.send_control.publish(ctrl)
        self.send_reference.publish(reference)

    def publish_setpoint(self):

        if self.control_mode == "position":
            setpoint_pos = PositionTarget()
            setpoint_pos.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_pos.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.IGNORE_YAW_RATE

            setpoint_pos.position.x = self.x
            setpoint_pos.position.y = self.y
            setpoint_pos.position.z = self.z

            if self.iteration == 1:
                self.iteration = 2
                self.set_mode('OFFBOARD')
                print("x y z: ")

            if self.iteration == 0:
                self.iteration = 1
                self.arm()

            setpoint_pos.header.stamp = rospy.Time.now()
            self.send_goal_mavros.publish(setpoint_pos)

        if self.control_mode == "velocity":
            setpoint_vel = PositionTarget()
            setpoint_vel.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            # + PositionTarget.IGNORE_PZ
            setpoint_vel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY \
                                 + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                 + PositionTarget.IGNORE_YAW_RATE

            setpoint_vel.position.z = self.z
            setpoint_vel.velocity.x = self.control_output
            setpoint_vel.velocity.y = 0
            setpoint_vel.velocity.z = 0

            self.send_goal_mavros.publish(setpoint_vel)
            self.rate.sleep()

    def get_user_input(self):
        while not rospy.is_shutdown():
            try:
                input_str = input("Enter 'x y z' for position or 'v' for velocity control: ").strip()
                if input_str.lower() == 'v':
                    self.control_mode = "velocity"
                    self.force_setpoint = float(input("Enter desired force setpoint in x direction: "))
                else:
                    self.x, self.y, self.z = map(float, input_str.split())
                    self.control_mode = "position"
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
