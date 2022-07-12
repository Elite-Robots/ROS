#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointStatePublisher():
    """
    关节状态发布者
    """

    def __init__(self) -> None:
        rospy.loginfo("JointStatePublisher is started...")
        self.joint_states_publisher_ = rospy.Publisher(
            'joint_states', JointState, queue_size=10)
        self.joint_name = rospy.get_param("~joint_name")
        self.joint_state_ = JointState()
        self.joint_state_.name = self.joint_name
        self.joint_state_.header = Header()

    def spin(self) -> None:
        """
        循环发布数据
        """
        self.update_joint_state()
        self.joint_states_publisher_.publish(self.joint_state_)

    def update_joint_state(self) -> None:
        """
        从SDK获取数据，并更新关节数据
        """
        self.joint_state_.header.stamp = rospy.Time.now()
        self.joint_state_.position.clear()
        self.joint_state_.velocity.clear()
        self.joint_state_.effort.clear()

        for joint in self.elite_robot.monitor_info.machinePos[:6]:  # pylint: disable=E1101
            self.joint_state_.position.append(math.radians(joint))
        
        for speed in self.elite_robot.monitor_info.joint_speed[:6]:  # pylint: disable=E1101
            self.joint_state_.velocity.append(math.radians(speed))
        self.joint_state_.effort = self.elite_robot.monitor_info.torque[
            :6]  # pylint: disable=E1101
