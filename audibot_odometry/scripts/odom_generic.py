#! /usr/bin/env python

import rospy
from audibot_cfg.vehicle_ns import AudiBotNS
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

audibot_obj = AudiBotNS()
audibot_obj.parse_args()
audibot_obj.exec_odom_cfg()
node_name, model_name, frame_id, topic_name = audibot_obj.get_odom_cfg()
rospy.init_node(node_name, anonymous=True)
odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=10)
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
odom = Odometry()
header = Header()
header.frame_id = frame_id
model = GetModelStateRequest()
model.model_name = model_name
r = rospy.Rate(10)
while not rospy.is_shutdown():
    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist
    header.stamp = rospy.Time.now()
    odom.header = header
    odom_pub.publish(odom)
    r.sleep()