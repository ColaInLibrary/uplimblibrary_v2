#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/11/11 下午3:37
# @Author  : AN Hao
# @File    : task_definition.py
# @Description :
import json
import time
from enum import Enum


class TaskDefinition:
    class CommandType(Enum):
        MOVEJ = "MOVEJ"
        MOVEL = "MOVEL"
        SET_VELOCITY = "SET_VELOCITY"
        GET_STATUS = "GET_STATUS"
        # ... 其他指令

    def __init__(self, socket):
        self.socket = socket

    def create_command(self, cmd_type, **params):
        return {
            "cmd": cmd_type.value,
            "id": int(time.time() * 1000),
            "timestamp": time.time(),
            "params": params
        }

    #  获取机器人运动
    def getDriverInfo(self):
        pass

    def sendDriverInfo(self, desired_info):
        pass

    # 发送运动指令
    def moveL(self, positions, velocity=50.0, acceleration=100.0, blocking=True, arm_type=1):
        cmd = self.create_command(
            self.CommandType.MOVEL,
            q=positions,
            speed=velocity,
            acceleration=acceleration,
            blocking=blocking,
            arm_type=arm_type
        )
        self.socket.send_string(json.dumps(cmd))
        return self.socket.recv_string()

    # 发送关节运动
    def moveJ(self, joint_positions, velocity=30.0, acceleration=10.0, blocking=False, arm_type=1):
        cmd = self.create_command(
            self.CommandType.MOVEJ,
            q=joint_positions,
            speed=velocity,
            acceleration=acceleration,
            asynchronous=blocking,
            arm_type=arm_type
        )
        self.socket.send_string(json.dumps(cmd))
        return self.socket.recv_string()
