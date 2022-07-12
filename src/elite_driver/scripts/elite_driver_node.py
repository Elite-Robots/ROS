#!/usr/bin/env python3
# _*_ coding:utf-8 _*_
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import rospy
from elite_driver.elite_driver import EliteDriver


def main() -> None:
    """主函数，循环发布"""
    rospy.init_node('elite_driver', disable_signals=True)
    elite_driver = EliteDriver()
    elite_driver.init_ec_sdk()
    loop_rate = rospy.get_param("~/loop_rate", default=125)
    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        elite_driver.spin()
        rate.sleep()


if __name__ == "__main__":
    main()
