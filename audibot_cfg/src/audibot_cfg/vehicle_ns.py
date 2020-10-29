#! /usr/bin/env python

#############################################################
# File responsible for configuring the namespace parameters #
# related to the audibot vehicle(s)                         #
#############################################################

import sys

AUDIBOT_NS_LIST = ['none', 'audi', 'blue', 'orange']

class AudiBotNS:

    def parse_args(self):
        default_ns = 'audi'
        args_list = list(sys.argv)
        if len(args_list) <= 1:
            audibot_ns = default_ns
        else:
            args_list.pop(0)
            audibot_ns = args_list[0]
            if audibot_ns not in AUDIBOT_NS_LIST:
                audibot_ns = AUDIBOT_NS_LIST[0]
        self._audibot_ns = audibot_ns
        return audibot_ns

    def set_ns(self, ns=''):
        if ns != '':
            self._audibot_ns = ns

    def get_ns(self):
        return self._audibot_ns

    def exec_odom_cfg(self):
        self._odom_obj = AudiBotOdom()
        self._odom_obj.set_cfg(self._audibot_ns)

    def get_odom_cfg(self):
        return self._odom_obj.get_cfg()

    def exec_llc_cfg(self):
        self._llc_obj = AudiBotLlc()
        self._llc_obj.set_cfg(self._audibot_ns)

    def get_llc_cfg(self):
        return self._llc_obj.get_cfg()


class AudiBotOdom:

    def set_cfg(self, ns):
        self._odom_node_name        = ns + '_odom'
        self._gazebo_model_name     = ns
        self._gazebo_frame_id       = '/' + ns + '/base_link'
        self._odom_topic_name       = '/' + ns + '/odom'

    def get_cfg(self):
        return self._odom_node_name, self._gazebo_model_name, self._gazebo_frame_id, self._odom_topic_name


class AudiBotLlc:

    def set_cfg(self, ns):
        self._twistlat_node_name    = ns + '_twistlat'
        self._twistlot_node_name    = ns + '_twistlot'
        self._cmd_vel_topic_name    = ns + '/cmd_vel'
        self._thrtl_topic_name      = '/' + ns + '/throttle_cmd'
        self._brake_topic_name      = '/' + ns + '/brake_cmd'
        self._steer_topic_name      = '/' + ns + '/steering_cmd'

    def get_cfg(self):
        return self._twistlat_node_name, self._twistlot_node_name, self._cmd_vel_topic_name, self._thrtl_topic_name, self._brake_topic_name, self._steer_topic_name