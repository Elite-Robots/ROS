#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import rospy
from elite_msgs.srv import StopMove, StopMoveResponse, StopMoveRequest


class StopMoveService():
    """
    停止运动服务
    """

    def __init__(self) -> None:
        rospy.loginfo("StopMoveService is started...")
        self.robot_servo_on_server = rospy.Service(
            "stop_move", StopMove, self.handle_stop_move_)

    def handle_stop_move_(self, req: StopMoveRequest) -> StopMoveResponse:  # pylint: disable=W0613
        """处理停止运动"""
        res = StopMoveResponse()
        print("recv Stop Move")
        result_ = self.elite_robot.stop()  # pylint: disable=E1101
        res = result_
        return res
