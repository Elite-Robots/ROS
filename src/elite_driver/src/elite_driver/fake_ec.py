#!/usr/bin/env python3
"""
################################################################################
# Copyright(C) 2022-2027 Elite. All Rights Reserved.
# @brief : 简介
# @author: Elite
################################################################################
"""
import json
import time
from typing import List, Any, Optional
import queue
import threading
import rospy
from elite._baseec import BaseEC
from elite._monitor import ECMonitorInfo


class FakeEc():
    """
    虚拟机械臂
    """

    def __init__(self, ip_address: str, auto_connect: bool = False) -> None:
        self.ip_address = ip_address
        self.auto_connect = auto_connect
        self.current_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_torques = [0.0, 0.0, 0.0, 2.0, 2.0, 2.0]
        self.monitor_info = ECMonitorInfo()
        self.monitor_info_()
        self.runing_event = threading.Event()
        self.trjectory_queue = queue.Queue()

    def monitor_info_(self):
        """主动获取数据信息"""
        self.monitor_info.analog_ioInput = [1.0, 1.0]
        self.monitor_info.analog_ioOutput = [1.0, 1.0]
        self.monitor_info.collision = 1
        self.monitor_info.autorun_cycleMode = 1
        self.monitor_info.can_motor_run = 1
        self.monitor_info.digital_ioInput = 1
        self.monitor_info.digital_ioOutput = 1
        self.monitor_info.emergencyStopState = 1
        self.monitor_info.joint_speed = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.monitor_info.jointacc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.monitor_info.motor_speed = [10, 10, 10, 10, 10, 10]
        self.monitor_info.machinePos = [10, 10, 10, 10, 10, 10]
        self.monitor_info.machinePose = [10, 10, 10, 10, 10, 10]
        self.monitor_info.machineUserPose = [10, 10, 10, 10, 10, 10]
        self.monitor_info.machineFlangePose = [10, 10, 10, 10, 10, 10]
        self.monitor_info.machineUserFlangePose = [10, 10, 10, 10, 10, 10]
        self.monitor_info.robotMode = 1
        self.monitor_info.robotState = 1
        self.monitor_info.servoReady = 1
        self.monitor_info.tcp_speed = 1.0
        self.monitor_info.torque = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.monitor_info.tcpacc = 1.0

    def monitor_thread_run(self) -> bool:
        """模拟线程"""
        return

    def forward_kinematic(self, joint: List[float]) -> List[float]:
        """正向运动学"""
        print(joint)
        result = [5.00, 4.00, 3.00, 2.00, 1.00, 6.00]
        return result

    def inverse_kinematic(self, cart: List[float], ref: List[float]) -> List[float]:
        """逆向运动学"""
        print(f"cart: {cart}, ref: {ref}")
        return [5.00, 4.00, 3.00, 2.00, 1.00, 6.00]

    def move_joint(self, joint, speed, acc, dec) -> bool:
        """关节移动"""
        return True

    def move_line(self, cart_point, speed, speed_type, acc, dec) -> bool:
        """笛卡尔空间直线运动"""
        return True

    def robot_servo_on(self) -> bool:
        """伺服上电"""
        return True

    def set_digital_io(self, addr, value) -> bool:
        """设置数字输出"""
        return True

    def set_analog_io(self, addr, value) -> bool:
        """设置模拟输出"""
        return True

    def stop(self) -> bool:
        """停止运动"""
        return True

    def ml_init(self, length: int, point_type: int, ref_joint: list, ref_frame: list, ret_flag: int) -> bool:
        """初始化带时间戳轨迹文件运动
           #!传输的第一个点位的时间戳必须为0

        Args
        ----
            length (int): 点位数量
            point_type (int): 点位类型,0: 关节,1: 位姿
            ref_joint (list): 参考关节角,如果点位类型为位姿,参考点为第一个点的逆解参考点
            ref_frame (list): 用户坐标系,如果为基座坐标系全为0
            ret_flag (int): 添加点位指令是否有返回值,0无,1有

        Returns
        -------
            bool: True操作成功,False操作失败
        """
        return self.send_CMD("start_push_pos", {"path_lenth": length, "pos_type": point_type, "ref_joint_pos": ref_joint, "ref_frame": ref_frame, "ret_flag": ret_flag})

    def send_CMD(self, cmd: str, params: Optional[dict] = None, id: int = 1, ret_flag: int = 1) -> Any:
        """向8055发送指定命令

        Args
        ----
            cmd (str): 指令
            params (Dict[str,Any], optional): 参数. Defaults to None.
            id (int, optional): id号. Defaults to 1.
            ret_flag (int, optional): 发送数据后是否接收数据,0不接收,1接收. Defaults to 1.

        Returns
        -------
            Any: 对应指令返回的信息或错误信息
        """
        if(not params):
            params = {}
        else:
            params = json.dumps(params)
        sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(
            cmd, params, id)+"\n"

        rospy.loginfo(sendStr)

    def ml_push(self, time_stamp: float, pos: list) -> bool:
        """添加带时间戳文件运动点位

        Args
        ----
            time_stamp (float): 时间戳,大于等于0,且递增单位: s
            pos (list): 点位数据

        Returns
        -------
            bool: True操作成功,False操作失败
        """
        self.trjectory_queue.put({"timestamp": time_stamp, "pos": pos})
        return self.send_CMD("push_pos", {"timestamp": time_stamp, "pos": pos}, ret_flag=0)

    def ml_end_push(self) -> bool:
        """停止添加时间戳点位,并返回push结果,push结果正确返回True

        Returns
        -------
            bool: True操作成功,False操作失败
        """
        return self.send_CMD("stop_push_pos")

    def ml_check_push_result(self) -> BaseEC.MlPushResult:
        """检查push结果

        Returns
        -------
            MlPushResult: 0:push点位和时间戳正确,-1:点位长度不符,-2:点位格式错误,-3:时间戳不规范
        """
        return BaseEC.MlPushResult(self.send_CMD("check_trajectory"))

    def ml_flush(self) -> bool:
        """清空缓存

        Returns
        -------
            bool: True操作成功,False操作失败
        """
        return self.send_CMD("flush_trajectory")

    def ml_run(self, speed_percent: float = 0.1) -> bool:
        """开始运行带时间戳的轨迹文件

        Args
        ----
            speed_percent (float, optional): 轨迹速度百分比,即以原始速度乘百分比的速度运动.单位 %, 范围>=0.1. Defaults to 0.1.

        Returns
        -------
            bool: True操作成功,False操作失败
        """
        self.runing_event.clear()

        def run_trajectory():
            last_timestamp = 0.0
            while self.trjectory_queue.qsize()>0:
                pose = self.trjectory_queue.get()
                time.sleep(pose['timestamp']-last_timestamp)
                self.current_joint = pose['pos'][:6]
                last_timestamp = pose['timestamp']
                print(self.current_joint)
            self.runing_event.set()
        threading.Thread(target=run_trajectory).start()

        return self.send_CMD("start_trajectory", {"speed_percent": speed_percent})

    def wait_stop(self):
        """等待被set"""
        self.runing_event.wait()
