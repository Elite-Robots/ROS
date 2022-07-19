#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import operator
import rospy
from elite_driver.elite_driver import EliteDriver


def test_elite_driver() -> None:
    """
    测试艾力特SDK驱动是否正常
    """
    rospy.init_node("test_elite_driver")
    rospy.set_param("~ip_address", "192.168.1.200")
    rospy.set_param("~auto_connect", False)
    rospy.set_param("~joint_name", ["1", "2", "3"])
    ec_driver = EliteDriver()
    assert ec_driver.ip_address == "192.168.1.200"
    assert ec_driver.auto_connect is False
    assert operator.eq(ec_driver.joint_name, ["1", "2", "3"])
    rospy.signal_shutdown("test_finish")


if __name__ == "__main__":
    test_elite_driver()
