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
from elite_msgs.srv import CartMove, CartMoveRequest
from elite_driver.cart_move import CartMoveService
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

def test_cart_move_server(ec_real) -> None:  # pylint: disable=W0621
    """
    测试笛卡尔空间运行通信是否正常
    """
    ec_robot = ec_real
    rospy.init_node("test_cart_move_server")
    cart_move_server = CartMoveService()
    cart_move_server.elite_robot = ec_robot
    rospy.wait_for_service("cart_move_server")
    cart_move_client = rospy.ServiceProxy("cart_move_server", CartMove)
    req = CartMoveRequest()
    current_point = ec_robot.get_joint()
    current_point[4] += 10
    req.target_joint = current_point
    req.speed = 10
    req.acc = 5
    req.dec = 5
    req.is_blocking = False
    res = cart_move_client(req)
    assert res.result is True

if __name__ == "__main__":
    elite_robot = EC(ip='192.168.1.200', auto_connect=True)
    elite_robot.monitor_thread_run()
    test_cart_move_server(elite_robot)
