#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
import rospy
import time
from elite_driver.cart_move import CartMoveService
from elite_driver.forward_kinematic import ForwardKinematicService
from elite_driver.inverse_kinematic import InverseKinematicService
from elite_driver.joint_move import JointMoveService
from elite_driver.joint_states import JointStatePublisher
from elite_driver.robot_servo_on import RobotServoOnService
from elite_driver.set_analog_io import SetAnalogIOService
from elite_driver.set_digital_io import SetDigitalIOService
from elite_driver.stop_move import StopMoveService
from elite_driver.robot_state import RobotStatePublisher
from elite_driver.joint_trajectory_move import JointTrajectoryMoveTService
from elite_driver.joint_trajectory_movel import JointTrajectoryMoveLService


from elite_driver.fake_ec import FakeEc
from elite import EC


class EliteDriver(JointStatePublisher, JointTrajectoryMoveTService,
                  JointTrajectoryMoveLService, ForwardKinematicService,
                  InverseKinematicService, JointMoveService, CartMoveService, RobotServoOnService,
                  SetAnalogIOService, StopMoveService, SetDigitalIOService, RobotStatePublisher):
    """
    艾力特机械臂ROS驱动层实现
    """

    def __init__(self) -> None:
        rospy.loginfo("EliteDriver is started...")
        JointStatePublisher.__init__(self)
        JointMoveService.__init__(self)
        ForwardKinematicService.__init__(self)
        InverseKinematicService.__init__(self)
        CartMoveService.__init__(self)
        RobotServoOnService.__init__(self)
        SetAnalogIOService.__init__(self)
        SetDigitalIOService.__init__(self)
        StopMoveService.__init__(self)
        RobotStatePublisher.__init__(self)
        JointTrajectoryMoveLService.__init__(self)
        JointTrajectoryMoveTService.__init__(self)

        self.elite_robot = None
        self.ip_address = rospy.get_param("~ip_address")
        self.auto_connect = rospy.get_param("~auto_connect")
        self.use_fake = rospy.get_param("~use_fake")

    def init_ec_sdk(self) -> None:
        """
        初始化SDK
        """
        rospy.loginfo(f"Use FakeEc is {self.use_fake}")
        if self.use_fake:
            self.elite_robot = FakeEc(
                ip_address=self.ip_address, auto_connect=self.auto_connect)
        else:
            self.elite_robot = EC(
                ip=self.ip_address, auto_connect=self.auto_connect)
            # 启动后从上一运行状态退出
            if self.elite_robot.state == EC.RobotState.PLAY:
                self.elite_robot.stop()
            self.elite_robot.robot_servo_on()
        self.elite_robot.monitor_thread_run()
        # 防止取出None,保证运行正确
        while self.elite_robot.monitor_info.machinePos[0] == None:
            rospy.loginfo("Monitor is not start,wait...")
            time.sleep(1)
        rospy.loginfo("Robot startup success...")


    def spin(self) -> None:  # pylint: disable=W0235
        """以一定速率遍历节点"""
        JointStatePublisher.spin(self)
        RobotStatePublisher.spin(self)
