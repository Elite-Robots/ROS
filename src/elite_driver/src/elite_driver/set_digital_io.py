'''
Author: Elite_zhangjunjie
CreateDate: 
LastEditors: Elite_zhangjunjie
LastEditTime: 2022-07-19 18:15:20
Description: 
'''
#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: elite
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

    def handle_set_digital_io_(self, req):
        """处理设置请求"""
        res = SetIOResponse()
        result = self.elite_robot.set_digital_io(  # pylint: disable=E1101
            req.address, req.value)
        res.result = result
        print(f"result:{result},type:{type(result)}")
        return res
