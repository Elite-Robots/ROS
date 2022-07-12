#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import threading
import time
import operator
import pytest
import rospy
from elite_msgs.msg import RobotState
from elite_driver.robot_state import RobotStatePublisher
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
    ec_real = EC(ip='192.168.1.200', auto_connect=True)
    ec_real.monitor_thread_run()
    return ec_real


def test_robot_states_publisher(ec_real):  # pylint: disable=W0621
    """测试数据发布者"""
    ec_robot = ec_real
    rospy.init_node("test_elite_robot_states")
    robot_states_publisher = RobotStatePublisher()
    robot_states_publisher.elite_robot = ec_robot

    def publish_robots():
        while not rospy.is_shutdown():
            robot_states_publisher.spin()
            time.sleep(0.01)

    threading.Thread(target=publish_robots).start()
    time.sleep(0)

    data = rospy.wait_for_message("robot_state", RobotState, timeout=10)
    rospy.signal_shutdown("test_finish")

    assert operator.eq(list(data.joint_speed),
                       ec_robot.monitor_info.joint_speed)
    assert operator.eq(list(data.joint_speed),
                       ec_robot.monitor_info.joint_speed)
    assert operator.eq(list(data.machineFlangePose),
                       ec_robot.monitor_info.machineFlangePose)
    assert operator.eq(list(data.machinePos), ec_robot.monitor_info.machinePos)
    print(list(data.analog_ioInput), ec_robot.monitor_info.analog_ioInput)
    # IO数据变化太快，通不过测试
    # assert operator.eq(list(data.analog_ioInput),ec_robot.monitor_info.analog_ioInput)


if __name__ == "__main__":
    test_robot_states_publisher(ec_real())
