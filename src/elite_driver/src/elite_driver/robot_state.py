#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import rospy
from std_msgs.msg import Header
from elite_msgs.msg import RobotState


class RobotStatePublisher():
    """
    关节状态发布者
    """

    def __init__(self) -> None:
        rospy.loginfo("RobotStatePublisher is started...")
        self.robot_state_publisher_ = rospy.Publisher(
            'robot_state', RobotState, queue_size=10)
        self.robot_state_ = RobotState()
        self.robot_state_.header = Header()

    def spin(self) -> None:
        """
        循环发布数据
        """
        self.update_robot_state()
        self.robot_state_publisher_.publish(self.robot_state_)

    def update_robot_state(self) -> None:
        """
        从SDK获取数据，并更新数据
        """
        self.robot_state_.header.stamp = rospy.Time.now()
        
        self.robot_state_.analog_ioInput = self.elite_robot.monitor_info.analog_ioInput
        self.robot_state_.analog_ioOutput = self.elite_robot.monitor_info.analog_ioOutput
        self.robot_state_.autorun_cycleMode = self.elite_robot.monitor_info.autorun_cycleMode

        self.robot_state_.can_motor_run = self.elite_robot.monitor_info.can_motor_run
        self.robot_state_.collision = self.elite_robot.monitor_info.collision

        self.robot_state_.digital_ioInput = self.elite_robot.monitor_info.digital_ioInput
        self.robot_state_.digital_ioOutput = self.elite_robot.monitor_info.digital_ioOutput

        self.robot_state_.emergencyStopState = self.elite_robot.monitor_info.emergencyStopState

        self.robot_state_.joint_speed = self.elite_robot.monitor_info.joint_speed
        self.robot_state_.jointacc = self.elite_robot.monitor_info.jointacc

        self.robot_state_.motor_speed = self.elite_robot.monitor_info.motor_speed
        self.robot_state_.machineUserFlangePose = self.elite_robot.monitor_info.machineFlangePose
        self.robot_state_.machineUserPose = self.elite_robot.monitor_info.machineUserPose
        self.robot_state_.machinePos = self.elite_robot.monitor_info.machinePos
        self.robot_state_.machinePose = self.elite_robot.monitor_info.machinePose
        self.robot_state_.machineFlangePose = self.elite_robot.monitor_info.machineFlangePose

        self.robot_state_.robotMode = self.elite_robot.monitor_info.robotMode
        self.robot_state_.robotState = self.elite_robot.monitor_info.robotState

        self.robot_state_.servoReady = self.elite_robot.monitor_info.servoReady

        self.robot_state_.tcp_speed = self.elite_robot.monitor_info.tcp_speed
        self.robot_state_.torque = self.elite_robot.monitor_info.torque
        self.robot_state_.tcpacc = self.elite_robot.monitor_info.tcpacc
