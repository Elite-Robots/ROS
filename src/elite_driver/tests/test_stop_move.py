#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import pytest
import rospy
from elite_msgs.srv import StopMove, StopMoveRequest
from elite_driver.stop_move import StopMoveService
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

def test_stop_move_server(ec_real):  # pylint: disable=W0621
    """测试停止运动服务"""
    ec_robot = ec_real
    rospy.init_node("test_stop_move_server")
    stop_move_server = StopMoveService()
    stop_move_server.elite_robot = ec_robot
    rospy.wait_for_service("stop_move")
    stop_move_client = rospy.ServiceProxy("stop_move", StopMove)
    req = StopMoveRequest()
    res = stop_move_client(req)
    assert res.result is True
    print("res", res)

if __name__ == "__main__":
    elite_robot = EC(ip='192.168.1.200', auto_connect=True)
    elite_robot.monitor_thread_run()
    test_stop_move_server(elite_robot)
