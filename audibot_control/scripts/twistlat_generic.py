#! /usr/bin/env python

import rospy
from audibot_cfg.vehicle_ns import AudiBotNS
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

global steer_pub

def twist_callback(twist_msg):
    global steer_pub
    raw_ang = twist_msg.angular.z
    steer_cmd = raw_ang * 17.3
    steer_pub.publish(Float64(steer_cmd))


def twist_lat_control(twistlat_node_name, cmd_vel_topic_name, steer_topic_name):
    global steer_pub
    rospy.init_node(twistlat_node_name, anonymous=True)
    steer_pub = rospy.Publisher(steer_topic_name, Float64, queue_size=10)
    rospy.Subscriber(cmd_vel_topic_name, Twist, twist_callback)
    steer_pub.publish(Float64(0.0))
    rospy.spin()


if __name__ == '__main__':
    audibot_obj = AudiBotNS()
    audibot_obj.parse_args()
    audibot_obj.exec_llc_cfg()
    twistlat_node_name, _, cmd_vel_topic_name, _, _, steer_topic_name = audibot_obj.get_llc_cfg()
    try:
        twist_lat_control(twistlat_node_name, cmd_vel_topic_name, steer_topic_name)
    except rospy.ROSInterruptException:
        pass