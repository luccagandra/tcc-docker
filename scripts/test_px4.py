#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import time

def publish_setpoint():
    # Inicializa o nó ROS
    rospy.init_node('setpoint_position_publisher', anonymous=True)
    
    # Cria um publicador para o tópico /mavros/setpoint_position/local
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    # Define a taxa de publicação em 30 Hz
    rate = rospy.Rate(30)
    
    # Inicializa a mensagem PoseStamped
    setpoint_msg = PoseStamped()
    setpoint_msg.header.seq = 0
    setpoint_msg.header.frame_id = ''
    setpoint_msg.pose.position.x = 0.0
    setpoint_msg.pose.position.y = 0.0
    setpoint_msg.pose.position.z = 2.0
    setpoint_msg.pose.orientation.x = 0.0
    setpoint_msg.pose.orientation.y = 0.0
    setpoint_msg.pose.orientation.z = 0.0
    setpoint_msg.pose.orientation.w = 0.0
    
    # Publica a mensagem a cada 1/30 segundos (30 Hz)
    while not rospy.is_shutdown():
        setpoint_msg.header.stamp = rospy.Time.now()
        pub.publish(setpoint_msg)
        setpoint_msg.header.seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_setpoint()
    except rospy.ROSInterruptException:
        pass