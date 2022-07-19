#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import rospy
from elite_msgs.srv import InverseKinematic, InverseKinematicResponse
import transforms3d as tfs
import math

class InverseKinematicService():  # pylint: disable=R0903
    """
    逆运动学服务接口，提供逆向运动学服务
    """

    def __init__(self) -> None:
        rospy.loginfo("InverseKinematicService is started...")
        self.forward_kinematic_server = rospy.Service(
            "inverse_kinematic", InverseKinematic, self.handle_inverse_kinematic)

    def handle_inverse_kinematic(self, req) -> None:
        """
        处理请求函数
        """
        self.res = InverseKinematicResponse()
        target_point = []
        target_point.append(req.pose.position.x)
        target_point.append(req.pose.position.y)
        target_point.append(req.pose.position.x)

        euler_angle = tfs.euler.quat2euler(
            [req.pose.orientation.w, req.pose.orientation.x,
             req.pose.orientation.y, req.pose.orientation.z])
        for angle in euler_angle:
          target_point.append(math.degrees(angle))

        result = self.elite_robot.get_inverse_kinematic(  # pylint: disable=E1101
            target_point, req.ref_joint,unit_type=0)
        if type(result) == tuple:
            res.result = False
        else:
            res.joint = result
            res.result = True
        return res
