#! /usr/bin/env python


import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


rospy.init_node('odom_blue_audi')
odom_pub = rospy.Publisher('/blue/odom', Odometry, queue_size=10)
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
odom = Odometry()
header = Header()
header.frame_id = '/blue/base_link'
model = GetModelStateRequest()
model.model_name = 'blue'
r = rospy.Rate(10)
while not rospy.is_shutdown():
    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist
    header.stamp = rospy.Time.now()
    odom.header = header
    odom_pub.publish(odom)
    r.sleep()
