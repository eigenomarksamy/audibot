#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


max_speed = 3.6		#TODO: Find the actual maximum speed
aucmd_topic = '/blue/cmd_vel'
thrtl_topic = '/blue/throttle_cmd'
brake_topic = '/blue/brake_cmd'
thrtl_pub = rospy.Publisher(thrtl_topic, Float64, queue_size=10)
brake_pub = rospy.Publisher(brake_topic, Float64, queue_size=10)


def twist_callback(twist_msg):
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


def twist_lot_control():
	rospy.init_node('twist_blue_audi', anonymous=True)
	rospy.Subscriber(aucmd_topic, Twist, twist_callback)
	thrtl_pub.publish(Float64(0.0))
	brake_pub.publish(Float64(0.0))
	rospy.spin()


if __name__ == '__main__':
	try:
		twist_lot_control()
	except rospy.ROSInterruptException:
		pass
