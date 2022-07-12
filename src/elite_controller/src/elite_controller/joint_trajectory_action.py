"""docs"""
import math
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from elite_msgs.srv import JointTrajectoryMove, JointTrajectoryMoveRequest
from elite_msgs.srv import StopMove, StopMoveRequest
from elite_controller.interpolate_five import get_five_fun, get_path_fun


class JointTrajectoryAction():
    def __init__(self) -> None:
        print("new action server")
        self.action_server = actionlib.ActionServer("manipulator_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction, self.on_goal, self.on_cancel, auto_start=False)
        self.action_server.start()
        self.trajectory_service_name = rospy.get_param(
            "~trajectory_service_name", "joint_trajectory_movet")
        self.interpolate_sample_time = rospy.get_param(
            "~interpolate_sample_time", 0.008)

    def on_goal(self, goal_handle: actionlib.ServerGoalHandle):
        """收到目标，处理请求"""
        goal_handle.set_accepted()
        goal = goal_handle.get_goal()
        print(goal)
        #解析数据 
        points = goal.trajectory.points
        joints = []
        for joint_index in range(6):
            sum_time, f, s, a = get_path_fun(points, joint_index)
            jointi = []
            length = int(sum_time/self.interpolate_sample_time)
            for point_index in range(length):
                jointi.append(math.degrees(
                    f(self.interpolate_sample_time*point_index)))
            joints.append(jointi)

        length = len(joints[0])
        joint_trajectory_client = rospy.ServiceProxy(
            self.trajectory_service_name, JointTrajectoryMove)

        # 构造请求
        req = JointTrajectoryMoveRequest()
        req.length = length
        for point_index in range(length):
            req.time_stamp.append(point_index*self.interpolate_sample_time)
            # 添加关节1-6角度
            for i in range(6):
                req.joint.append(joints[i][point_index])
            req.joint.append(0.0)
            req.joint.append(0.0)
        req.is_blocking = True

        # 请求服务
        response = joint_trajectory_client(req)
        if response.result:
            goal_handle.set_succeeded()
        else:
            goal_handle.set_failed()

    def _stop_move(self) -> bool:
        """停止移动"""
        print("req Stop Move")
        stop_move_client = rospy.ServiceProxy("stop_move", StopMove)
        res = stop_move_client(StopMoveRequest())
        return res.result

    def on_cancel(self, goal_handle: actionlib.ServerGoalHandle):
        """执行过程中被取消"""
        print("==========================stop===========================")
        self._stop_move()
        goal_handle.set_aborted()
        goal_handle.set_canceled()
