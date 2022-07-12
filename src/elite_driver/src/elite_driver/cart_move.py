#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import rospy
# import transforms3d as tfs
from elite import EC
from elite_msgs.srv import CartMove, CartMoveResponse, CartMoveRequest


class CartMoveService():  # pylint: disable=R0903
    """
    机械臂空间笛卡尔运动服务
    """

    def __init__(self) -> None:
        rospy.loginfo("CartMoveService is started...")
        self.cart_move_server = rospy.Service(
            "cart_move_server", CartMove, self.handle_cart_move)
        self.res = CartMoveResponse()

    def handle_cart_move(self, req: CartMoveRequest) -> CartMoveResponse:
        """
        处理笛卡尔运动的服务回调函数，由ROS系统调用
        """
        target_joint =  req.target_joint
        speed = req.speed
        speed_type = req.speed_type
        acc = req.acc
        dec = req.dec
        is_block = req.is_blocking
        result_ = self.elite_robot.move_line(  # pylint: disable=E1101
            target_joint, speed, speed_type, acc, dec)
        if is_block:
            self.elite_robot.wait_stop()  # pylint: disable=E1101
        self.res.result = result_
        return self.res
