#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import pytest
import rospy
from elite_msgs.srv import JointMove, JointMoveRequest
from elite_driver.joint_move import JointMoveService
from elite_driver.fake_ec import FakeEc
from elite import EC


@pytest.fixture
def ec_fake() -> FakeEc:
    """
    生产虚拟机械臂，用于测试使用
    """
    fake_ec_ = FakeEc("123")
    return fake_ec_

@pytest.fixture
def ec_real() -> EC:
    """
    生产真实机械臂，用于实际测试
    """
    return EC(ip='192.168.1.200', auto_connect=True)

def test_joint_move_server(ec_real):  # pylint: disable=W0621
    """测试关节运动"""
    ec_robot = ec_real
    rospy.init_node("test_cart_move")
    joint_move_server = JointMoveService()
    joint_move_server.elite_robot = ec_robot
    rospy.wait_for_service("joint_move")
    joint_move_client = rospy.ServiceProxy("joint_move", JointMove)
    current_point = ec_robot.get_joint()
    current_point[5] += 20
    req = JointMoveRequest()
    req.target_joint = current_point
    req.speed = 0.8
    req.acc = 5
    req.dec = 10
    req.is_blocking = False
    res = joint_move_client(req)
    assert res.result
    print("res", res)


if __name__ == "__main__":
    elite_robot = EC(ip='192.168.1.200', auto_connect=True)
    elite_robot.monitor_thread_run()
    test_joint_move_server(elite_robot)
