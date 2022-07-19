#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import rospy
from elite_msgs.srv import RobotServoOn, RobotServoOnResponse


class RobotServoOnService(): # pylint: disable=R0903
    """机器人上伺服使能服务"""

    def __init__(self) -> None:
        rospy.loginfo("RobotServoOnService is started...")
        self.robot_servo_on_server = rospy.Service(
            "robot_servo_on", RobotServoOn, self.handle_robot_servo_on_)
        self.res = RobotServoOnResponse()

    def handle_robot_servo_on_(self, req):  # pylint: disable=W0613
        """处理机器人伺服上使能"""
        result_ = self.elite_robot.robot_servo_on()  # pylint: disable=E1101
        self.res = result_
        return self.res
