#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/9/13 上午11:20
# @Author  : AN Hao
# @File    : plot_send.py
# @Description :
import json

import zmq
import time
from typing import List, Union


class PlotJugglerSender:
    """
    ZMQ-based data sender for PlotJuggler
    """

    def __init__(self, port: int = 9873, topic: str = "test1"):
        """
        初始化ZMQ发布者

        :param port: ZMQ通信端口 (默认5555)
        :param topic: 数据主题名称 (默认"robot_data")
        """
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        self.topic = topic
        time.sleep(0.5)  # 等待订阅者连接
        print(f"PlotJuggler sender ready at port {port}")

    def _send_data(self, data):
        """内部发送方法"""
        self.socket.send(data)

    def send_joints(self,
                    angles: List[float],
                    velocities: List[float],
                    timestamp: Union[float, None] = None):
        """
        发送关节数据

        :param angles: 关节角度 (rad)
        :param velocities: 关节速度 (rad/s)
        :param timestamp: 可选时间戳 (默认自动生成)
        """

        data = {
            "timestamp": timestamp,
            "joints": {
                "angle": angles,
                "velocity": velocities
            }
        }
        data = json.dumps(data)
        self._send_data(data.encode())

    def send_end_effectors(self,
                           poses: List[List[float]],
                           velocities: List[List[float]],
                           timestamp: Union[float, None] = None):
        """
        发送末端执行器数据

        :param poses: 两个末端的位姿 [x,y,z, qx,qy,qz,qw]
        :param velocities: 两个末端的速度 [vx,vy,vz, wx,wy,wz]
        :param timestamp: 可选时间戳 (默认自动生成)
        """
        data = {
            "timestamp": timestamp,
            "end_effectors": {
                "pose": poses,
                "velocity": velocities
            }
        }
        data = json.dumps(data)
        self._send_data(data.encode())

    def send_all(self,
                 angles: List[float],
                 velocities: List[float],
                 poses: List[float],
                 ee_velocities: List[float],
                 err_pose: List[float],
                 timestamp: Union[float, None] = None):
        """
        一次性发送所有数据

        :param angles: 关节角度
        :param velocities: 关节速度
        :param poses: 两个末端位姿
        :param ee_velocities: 两个末端速度
        """
        data = {
            "timestamp": timestamp,
            "joints": {
                "angle": angles,
                "velocity": velocities
            },
            "end_effectors": {
                "pose": poses,
                "velocity": ee_velocities,
                "error": err_pose
            }
        }
        data = json.dumps(data)
        self._send_data(data.encode())

    def close(self):
        """关闭连接"""
        self.socket.close()
        self.context.term()
        print("PlotJuggler sender closed")

    def __del__(self):
        self.close()