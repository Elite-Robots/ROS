#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import rospy
from elite_msgs.srv import ForwardKinematic, ForwardKinematicResponse, ForwardKinematicRequest
import transforms3d as tfs


class ForwardKinematicService():  # pylint: disable=R0903
    """
    机械臂正运动学解计算服务
    """

    def __init__(self) -> None:
        rospy.loginfo("ForwardKinematicService is started...")
        self.forward_kinematic_server = rospy.Service(
            "forward_kinematic", ForwardKinematic, self.handle_forward_kinematic)
        self.res = ForwardKinematicResponse()

    def handle_forward_kinematic(self, req: ForwardKinematicRequest) -> None:
        """
        处理正运动学求解
        """
        result = self.elite_robot.get_forward_kinematic(  # pylint: disable=E1101
            req.joint)
        self.res.pose.position.x = result[0]
        self.res.pose.position.y = result[1]
        self.res.pose.position.z = result[2]
        quat = tfs.euler.euler2quat(result[3], result[4], result[5])
        self.res.pose.orientation.w = quat[0]
        self.res.pose.orientation.x = quat[1]
        self.res.pose.orientation.y = quat[2]
        self.res.pose.orientation.z = quat[3]
        return self.res
