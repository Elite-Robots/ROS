#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import rospy
from elite_msgs.srv import JointMove, JointMoveResponse, JointMoveRequest


class JointMoveService():  # pylint: disable=R0903
    """
    关节运动服务
    """

    def __init__(self) -> None:
        rospy.loginfo("JointMoveService is started...")
        self.cart_move_server = rospy.Service(
            "joint_move", JointMove, self.handle_joint_move)

    def handle_joint_move(self, req: JointMoveRequest) -> JointMoveResponse:
        """
        处理关节运动请求
        """
        res = JointMoveResponse()
        joint_point_ = req.target_joint
        speed_ = req.speed
        acc_ = req.acc
        dec_ = req.dec
        is_block_ = req.is_blocking
        result_ = self.elite_robot.move_joint(  # pylint: disable=E1101
            joint_point_, speed_, acc_, dec_)
        if is_block_:
            self.elite_robot.wait_stop()  # pylint: disable=E1101
        if type(result_) == bool:
            res.result = result_
        else:
            res.result = result_[0]
        return res
