#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import rospy
from elite_msgs.srv import SetIO, SetIOResponse


class SetDigitalIOService():
    """
    设置数字输出，指定io和值
    """

    def __init__(self) -> None:
        rospy.loginfo("SetDigitalIOService is started...")
        self.set_io_server = rospy.Service(
            "set_digital_io_server", SetIO, self.handle_set_digital_io_)
        self.res = SetIOResponse()

    def handle_set_digital_io_(self, req):
        """处理设置请求"""
        result = self.elite_robot.set_digital_io(  # pylint: disable=E1101
            req.address, req.value)
        self.res.result = result
        print(f"result:{result}")
        return self.res
