#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: 作者
################################################################################
"""
from time import time
import rospy
from elite_msgs.srv import JointTrajectoryMove, JointTrajectoryMoveResponse, JointTrajectoryMoveRequest
# from elite._moveml import EC
from elite import EC


class JointTrajectoryMoveLService():  # pylint: disable=R0903
    """关节轨迹运动服务"""

    def __init__(self) -> None:
        rospy.loginfo("JointTrajectoryMoveLService is started...")
        self.joint_trajectory_move_server = rospy.Service(
            'joint_trajectory_movel', JointTrajectoryMove, self.handle_trajectory_move_)

    def handle_trajectory_move_(self, request: JointTrajectoryMoveRequest):
        """处理运动请求"""
        response = JointTrajectoryMoveResponse()
        length = request.length
        time_stamp = list(request.time_stamp)
        joint = list(request.joint)

        self.elite_robot.ml_init(length=length, point_type=0, ref_joint=self.elite_robot.current_joint, ref_frame=[
                                 0, 0, 0, 0, 0, 0], ret_flag=0)
        for i in range(len(time_stamp)):
            temp_joint = joint[i*8:i*8+6]
            temp_time = time_stamp[i]
            self.elite_robot.ml_push(  # pylint: disable=E1101
                temp_time, temp_joint)
        print("end_push", self.elite_robot.ml_end_push())  # pylint: disable=E1101
        if self.elite_robot.ml_check_push_result() == EC.MlPushResult.CORRECT:
            self.elite_robot.ml_run(  # pylint: disable=E1101
                speed_percent=100.0)  # 全速运行，速度交由Moveit控制
        else:
            print("push result", self.elite_robot.ml_check_push_result())
        if request.is_blocking:
            self.elite_robot.wait_stop()  # pylint: disable=E1101

        response.result = True
        return response
