#! /usr/bin/env python

import rospy
from audibot_cfg.vehicle_ns import AudiBotNS
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

global thrtl_pub, brake_pub

def twist_callback(twist_msg):
    global thrtl_pub, brake_pub
    max_speed = 3.6
    raw_vel = twist_msg.linear.x
    if raw_vel > 0:
        raw_vel = max(raw_vel, max_speed)
        thrtl_cmd = raw_vel / max_speed
        brake_cmd = 0.0
    elif raw_vel < 0:
        raw_vel = min(raw_vel, (-1 * max_speed))
        thrtl_cmd = raw_vel / max_speed
        brake_cmd = 0.0
    else:
        thrtl_cmd = 0.0
        brake_cmd = 1.0
    thrtl_pub.publish(Float64(thrtl_cmd))
    brake_pub.publish(Float64(brake_cmd))


def twist_lot_control(twistlat_node_name, cmd_vel_topic_name, thtrl_topic_name, brake_topic_name):
    global thrtl_pub, brake_pub
    rospy.init_node(twistlot_topic_name, anonymous=True)
    thrtl_pub = rospy.Publisher(thtrl_topic_name, Float64, queue_size=10)
    brake_pub = rospy.Publisher(brake_topic_name, Float64, queue_size=10)
    rospy.Subscriber(cmd_vel_topic_name, Twist, twist_callback)
    steer_pub.publish(Float64(0.0))
    rospy.spin()


if __name__ == '__main__':
    audibot_obj = AudiBotNS()
    audibot_obj.parse_args()
    audibot_obj.exec_llc_cfg()
    _, twistlot_topic_name, cmd_vel_topic_name, thtrl_topic_name, brake_topic_name, _ = audibot_obj.get_llc_cfg()
    try:
        twist_lot_control(twistlot_topic_name, cmd_vel_topic_name, thtrl_topic_name, brake_topic_name)
    except rospy.ROSInterruptException:
        pass