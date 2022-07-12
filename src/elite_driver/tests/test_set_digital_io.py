#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
from ast import operator
import pytest
import rospy
import operator
from elite_msgs.srv import SetIO, SetIORequest
from elite_driver.set_digital_io import SetDigitalIOService
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


def test_set_digital_io_server(ec_real):  # pylint: disable=W0621
    """测试数字IO输出"""
    ec_robot = ec_real
    rospy.init_node("test_set_digital_io_server")
    set_digital_io_server = SetDigitalIOService()
    set_digital_io_server.elite_robot = ec_robot
    rospy.wait_for_service("set_digital_io_server")
    set_digital_io_client = rospy.ServiceProxy("set_digital_io_server", SetIO)
    req = SetIORequest()
    req.address = "Y0"
    req.value = 1
    res = set_digital_io_client(req)
    assert operator.eq(res.result, ec_robot.get_digital_io("Y0"))
    # assert res.result is True
    print("res", res)


if __name__ == "__main__":
    elite_robot = EC(ip='192.168.1.200', auto_connect=True)
    elite_robot.monitor_thread_run()
    # elite_robot.set_digital_io("Y0",1)
    test_set_digital_io_server(elite_robot)
