#!/usr/bin/env python
from __future__ import print_function
from turtle import turtles
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    计算当前的的机器人坐标是否和目标在一定的范围内，在表示到达目标点
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self,group_name = "manipulator"):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        # 实例化`RobotCommander`_对象. 该对象提供机器人的运动学模型信息和机器人当前的坐标。
        self.robot = moveit_commander.RobotCommander()
        
        # 初始化规划场景接口，该接口可以对规划场景进行编辑
        self.scene = moveit_commander.PlanningSceneInterface()

        # 根据规划组实例化moveit group，通过该对象可以进行运动规划并控制机器人运动
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # 创建一个轨迹发布话题，用于发布机器人的轨迹信息
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # 
        planning_frame = self.move_group.get_planning_frame()
        eef_link = self.move_group.get_end_effector_link()

        print(f"1.通过move_group获取到机器人的规划链为从：{planning_frame}->{eef_link}")
        print(f"2.通过move_group获取到机器人的当前关节坐标为：{self.move_group.get_current_joint_values()}")
        print(f"3.通过move_group获取到机器人的当前空间坐标为：{self.move_group.get_current_pose().pose}")


    def joint_move(self,joint_goal=[0,0,0,0,0,0]):
        move_group = self.move_group
        # 调用move_group 进行移动到目标位置
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def cart_move(self,pose_goal):
        print(f"收到移动到目标{pose_goal}指令")
        move_group = self.move_group
        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

def main():
    tutorial = MoveGroupPythonInterfaceTutorial("manipulator")
    tutorial.joint_move(joint_goal=[0.01,-pi/1.5,pi/1.5, -pi/2,pi/2.0,0])
    # 笛卡尔空间运动
    pose_goal = tutorial.move_group.get_current_pose().pose
    print(pose_goal)
    pose_goal.position.z -= 0.2
    print(pose_goal)
    input("按回车移动到目标位置")
    tutorial.cart_move(pose_goal)

    
if __name__=='__main__':
    main()