#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import threading
import time
import operator
import pytest
import math
import rospy
from sensor_msgs.msg import JointState
from elite_driver.joint_states import JointStatePublisher
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
    ec_real = EC(ip='192.168.1.200', auto_connect=True)
    ec_real.monitor_thread_run()
    return ec_real

def test_joint_states_publisher(ec_real):  # pylint: disable=W0621
    """测试关节数据发布者"""
    ec_robot = ec_real
    rospy.init_node("test_elite_joint_states")
    rospy.set_param(
        "~joint_name", ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
    joint_states_publisher = JointStatePublisher()
    joint_states_publisher.elite_robot = ec_robot

    def publish_joints():
        while not rospy.is_shutdown():
            joint_states_publisher.spin()
            time.sleep(0.01)

    threading.Thread(target=publish_joints).start()

    data = rospy.wait_for_message("joint_states", JointState, timeout=10)
    rospy.signal_shutdown("test_finish")
    print(list(data.position), ec_robot.current_joint)
    #TODO degree2raduian
    radians_pose = []
    for r in ec_robot.current_joint:
        radians_pose.append(math.radians(r))
    assert operator.eq(list(data.position), radians_pose)
    assert operator.eq(list(data.velocity), ec_robot.joint_speed)
    # 力矩变化太快
    # assert operator.eq(list(data.effort), ec_robot.joint_torques)

if __name__ == "__main__":
    elite_robot = EC(ip='192.168.1.200', auto_connect=True)
    elite_robot.monitor_thread_run()
    test_joint_states_publisher(elite_robot)
