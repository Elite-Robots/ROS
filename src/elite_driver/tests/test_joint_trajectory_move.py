#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import operator
import pytest
import rospy
from elite_msgs.srv import JointTrajectoryMove, JointTrajectoryMoveRequest
from elite_driver.joint_trajectory_move import JointTrajectoryMoveService
from elite_driver.fake_ec import FakeEc


@pytest.fixture
def ec_fake() -> FakeEc:
    """
    生产虚拟机械臂，用于测试使用
    """
    fake_ec_ = FakeEc("123")
    return fake_ec_


def test_joint_trajectory_server(ec_fake):  # pylint: disable=W0621
    """测试关节轨迹运动"""
    rospy.init_node("joint_trajectory_move")
    joint_trajectory_move_server = JointTrajectoryMoveService()
    joint_trajectory_move_server.elite_robot = ec_fake

    joint_trajectory_client = rospy.ServiceProxy(
        "joint_trajectory_move", JointTrajectoryMove)
    req = JointTrajectoryMoveRequest()
    req.length = 4
    req.time_stamp = [1.000, 2.001, 3.002, 4.003]
    req.joint = [1.0, 2.0, 3.0, 3.0, 3.0, 3.001, 0.0, 0.0,
                 1.0, 2.0, 3.0, 3.0, 3.0, 3.002, 0.0, 0.0,
                 1.0, 2.0, 3.0, 3.0, 3.0, 3.003, 0.0, 0.0,
                 1.0, 2.0, 3.0, 3.0, 3.0, 3.004, 0.0, 0.0
                 ]
    req.is_blocking = True
    assert joint_trajectory_client(req).result
    assert operator.eq(ec_fake.current_joint, [1.0, 2.0, 3.0, 3.0, 3.0, 3.004])

if __name__ == "__main__":
    test_joint_trajectory_server(FakeEc("123"))
