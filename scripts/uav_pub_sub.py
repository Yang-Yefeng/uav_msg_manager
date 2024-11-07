#! /usr/bin/python3
import os
import rospy

from uav_base import uav_base
from std_msgs.msg import Float32MultiArray, Bool, String, Float32


cur_path = os.path.dirname(os.path.abspath(__file__))


if __name__ == "__main__":
    rospy.init_node('uav_msg_manager')
    uav = uav_base(uav_name='uav0')
    
    
    while not rospy.is_shutdown():
        # uav.pos_pub.publish(Float32MultiArray(uav.pos))
        # uav.vel_pub.publish(Float32MultiArray(uav.vel))
        # uav.att_pub.publish(Float32MultiArray(uav.att))
        # uav.pqr_pub.publish(Float32MultiArray(uav.pqr))
        # uav.connect_pub.publish(Bool(uav.connected))
        # uav.armed_pub.publish(Bool(uav.armed))
        # uav.mode_pub.publish(String(uav.mode))
        # uav.voltage_pub.publish(Float32(uav.voltage))
        rospy.loginfo_once('uav_msg_manager started.')
        # print(uav.uav_msg.pos)
        uav.uav_msg_pub.publish(uav.uav_msg)
        uav.rate.sleep()
