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
from elite_msgs.srv import InverseKinematic, InverseKinematicRequest
from elite_driver.inverse_kinematic import InverseKinematicService
from elite_driver.fake_ec import FakeEc
from elite import EC
import transforms3d as tfs
import math


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
    # ec_real.get_inverse_kinematic(pose)
    return ec_real


def inverse_kinematic_server(ec_robot):  # pylint: disable=W0621
    """逆运动学"""
    rospy.init_node("test_inverse_kinematic")
    inverse_kinematic_server = InverseKinematicService()
    inverse_kinematic_server.elite_robot = ec_robot
    rospy.wait_for_service("inverse_kinematic")
    inverse_kinematic_client = rospy.ServiceProxy(
        "inverse_kinematic", InverseKinematic)
    req = InverseKinematicRequest()
    current_pose = ec_robot.current_pose
    req.pose.position.x = current_pose[0]
    req.pose.position.y = current_pose[1]
    req.pose.position.z = current_pose[2]
    # quat
    quat = tfs.euler.euler2quat(
        current_pose[3], current_pose[4], current_pose[5])
    req.pose.orientation.w = quat[0]
    req.pose.orientation.x = quat[1]
    req.pose.orientation.y = quat[2]
    req.pose.orientation.z = quat[3]
    req.ref_joint = ec_robot.current_joint
    req.ref_joint[0] += 1
    res = inverse_kinematic_client(req)
    assert res.joint
    assert res.result
    

def test_inverse_kinematic_server(ec_real):
    inverse_kinematic_server(ec_real)


if __name__ == "__main__":
    inverse_kinematic_server(EC(ip='192.168.1.200', auto_connect=True))
