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
import math
from elite_msgs.srv import ForwardKinematic, ForwardKinematicRequest
from elite_driver.forward_kinematic import ForwardKinematicService
from elite_driver.fake_ec import FakeEc
from elite import EC
import transforms3d as tfs


@pytest.fixture
def ec_fake() -> FakeEc:
    """
    生产虚拟机械臂，用于测试使用
    """
    fake_ec_ = FakeEc("123")
    return fake_ec_
    
@pytest.fixture
def ec_real() -> EC:
    ec_real = EC(ip='192.168.1.200', auto_connect=True)
    ec_real.monitor_thread_run()
    return ec_real

def forward_kinematic_server(ec_robot):  # pylint: disable=W0621
    """
    正向运动学测试
    """
    rospy.init_node("test_forward_kinematic")
    forward_kinematic_server = ForwardKinematicService()
    forward_kinematic_server.elite_robot = ec_robot
    rospy.wait_for_service("forward_kinematic")
    forward_kinematic_client = rospy.ServiceProxy(
        "forward_kinematic", ForwardKinematic)
    req = ForwardKinematicRequest()
    req.joint = ec_robot.current_joint
    res = forward_kinematic_client(req)
    euler = tfs.euler.quat2euler((
        res.pose.orientation.w, res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z))

    assert math.fabs(res.pose.position.x-ec_robot.current_pose[0]) < 0.01
    assert math.fabs(res.pose.position.y-ec_robot.current_pose[1]) < 0.01
    assert math.fabs(res.pose.position.z-ec_robot.current_pose[2]) < 0.01

    assert math.fabs(euler[0]-ec_robot.current_pose[3]) < 0.01
    assert math.fabs(euler[1]-ec_robot.current_pose[4]) < 0.01
    assert math.fabs(euler[2]-ec_robot.current_pose[5]) < 0.01


def test_real_arm(ec_real):
    """测试真实机械臂"""
    forward_kinematic_server(ec_real)


if __name__ == "__main__":
    ec = EC(ip='192.168.1.200', auto_connect=True)
    test_real_arm(ec)
