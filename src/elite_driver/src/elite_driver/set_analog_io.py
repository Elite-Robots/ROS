#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import rospy
from elite_msgs.srv import SetAnalogIO, SetAnalogIOResponse


class SetAnalogIOService():
    """设置模拟输出服务"""

    def __init__(self) -> None:
        rospy.loginfo("SetAnalogIOService is started...")
        self.set_io_server = rospy.Service(
            "set_analog_io", SetAnalogIO, self.handle_set_analog_io_)

    def handle_set_analog_io_(self, req):
        res = SetAnalogIOResponse()
        """处理设置模拟输出"""
        result = self.elite_robot.set_analog_output(  # pylint: disable=E1101
            req.address, req.value)
        res.result = result
        print(f"result:{result}")
        return res
