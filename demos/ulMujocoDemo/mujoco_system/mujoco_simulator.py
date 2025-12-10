#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/5/14 下午4:36
# @Author  : AN Hao
# @File    : genesis_simulator.py
# @Description :
import threading
import mujoco as mj
from mujoco.glfw import glfw
import mujoco.viewer
import numpy as np
from typing import List, Any
from .task_executor import TaskExecutor
from .keyboard_controller import KeyboardController
import time
from .plot_send import PlotJugglerSender
import json
import zmq
import sys


class MujocoSimulator:
    def __init__(self, xml_path: str, joint_names: List[str]):
        """
        Initialize Mujoco simulator

        Args:
            xml_path: Path to URDF model file
            joint_names: List of joint names
        """
        self.xml_path = xml_path
        self.joint_names = joint_names
        self.running = False
        self.sim_thread = None
        self.keyboard_controller = None
        self.task_executor = None

        # Genesis related objects
        self.scene = None
        self.cam = None
        self.nav = None
        self.dofs_idx = None
        self.dof = None
        self.model = None
        self.data = None

        # Controller related
        # self.measured_info = ul.InputInfo()
        # self.desired_info = ul.OutputInfo()

        # socket 初始化
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")

        # Initialize components
        self._initialize_genesis()
        self._setup_task_executor()
        self._setup_keyboard_controller()

        # 初始化PlotJuggler发送器
        self.pj_sender = PlotJugglerSender()
        self.loop_count = 0
        self.start_time = 0


    def _setup_task_executor(self):
        """Initialize task executor"""
        self.task_executor = TaskExecutor(self, self.socket, self.dof, self.dofs_idx)

    def _setup_keyboard_controller(self):
        """Setup keyboard controller with task mappings"""
        task_mapping = {
            '0': self.task_executor.go_zero,
            '1': self.task_executor.moveJ_example,
            # '2': self.task_executor.moveJ_path_example,
            # '3': self.task_executor.speedJ_example,
            # '4': self.task_executor.speedStop_example,
            # '5': self.task_executor.moveL_example,
            # '6': self.task_executor.speedL_example,
            # '7': self.task_executor.servoJ_example,
            # '8': self.task_executor.moveJ_pose_example,
            # '9': self.task_executor.go_home,
            # 'o': self.task_executor.smooth_example,
            # 't': self.task_executor.toppra_example
        }
        self.keyboard_controller = KeyboardController(
            task_mapping=task_mapping,
            quit_callback=self.stop  # 传递stop方法作为退出回调
        )

    def _initialize_genesis(self):
        """Initialize Genesis environment"""
        self.model = mj.MjModel.from_xml_path(self.xml_path)
        self.data = mj.MjData(self.model)
        self.dof = self.model.nv
        key_name = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_KEY, "home")
        pos_des = self.model.key_qpos[key_name]
        self.data.qpos = pos_des
        self.data.qvel = np.zeros_like(self.data.qvel)
        self.model.opt.timestep = 0.001



    def _simulation_loop(self):
        """Simulation loop (runs indefinitely until self.running is False)"""
        print("Simulation thread started...")

        # 启动被动查看器（新版API）
        # 可以按Tab或者 Shift+Tab 来打开左右面板
        with mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False) as viewer:
            # 调整相机视角（逐个修改属性）
            viewer.cam.lookat[:] = [0, 0, 0.5]  # 看向机器人的中心位置（x, y, z）
            viewer.cam.distance = 3.0  # 相机距离目标点的距离
            viewer.cam.azimuth = 180  # 水平旋转角度（0~360）
            viewer.cam.elevation = -20  # 俯仰角（-90~90，负值=俯视）

            last_real_time = time.time()
            self.start_time = last_real_time
            while viewer.is_running() and self.running:  # 直接循环检测运行状态
                current_real_time = time.time()
                elapsed_real_time = current_real_time - last_real_time
                last_real_time = current_real_time
                steps_to_simulate = int(elapsed_real_time / self.model.opt.timestep)
                # print(f"Simulating {steps_to_simulate} steps")

                for _ in range(steps_to_simulate):
                    # 物理仿真步进
                    self.position_control()
                    mujoco.mj_step(self.model, self.data)

                # 同步渲染（必需！）
                viewer.sync()

                # 可选：控制循环速度
                # time.sleep(0.001)  # 避免100% CPU占用
            viewer.close()

    def position_control(self):
        # start_time = time.time()
        q = self.data.qpos.copy()

        self.measured_info.act_pos[self.dof - 1] = q[0]  # waist
        self.measured_info.act_pos[self.dof - 2] = q[self.dof - 1]  # Neck
        self.measured_info.act_pos[self.dof - 3] = q[self.dof - 2]  # Neck
        for i in range(7):
            self.measured_info.act_pos[i] = q[i + 1 + 7]  # Left Arm
            self.measured_info.act_pos[i + 7] = q[i + 1]  # Right Arm

        # self.measured_info.act_pos[:self.dof] = q
        # ul.getDriverInfo(self.measured_info)
        # ul.sendDriverInfo(self.desired_info)

        self.data.qpos[0] = self.desired_info.cmd_pos[self.dof-1]  # waist
        self.data.qpos[self.dof - 1] = self.desired_info.cmd_pos[self.dof - 2]   # Neck
        self.data.qpos[self.dof - 2] = self.desired_info.cmd_pos[self.dof - 3]   # Neck
        for i in range(7):
            self.data.qpos[i + 1 + 7] = self.desired_info.cmd_pos[i]  # Left Arm
            self.data.qpos[i + 1] = self.desired_info.cmd_pos[i + 7]

        # 设置速度，避免仿真中发生漂移
        self.data.qvel = np.zeros_like(self.data.qvel)
        # end_time = time.time()
        # print(f"Time taken: {end_time - start_time}")
        self.pj_sender.send_all(angles=self.data.qpos.tolist(),
                                velocities=self.data.qvel.tolist(),
                                poses=np.zeros((2, 6)).tolist(),
                                ee_velocities=np.zeros((2, 6)).tolist(),
                                err_pose=np.zeros(6).tolist(),
                                timestamp=self.loop_count * 0.001 + self.start_time)
        self.loop_count += 1

    def start(self):
        """Start simulator and keyboard listener"""
        if self.running:
            print("Simulator is already running")
            return

        self.running = True

        # Start simulation thread
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()

        # Start keyboard controller (this will block)
        self.keyboard_controller.start()

        # Wait for simulation thread to finish
        self.sim_thread.join()
        print("Simulator start function exiting...")

    def stop(self):
        """Stop simulator and all components"""
        print("Stopping simulator...")
        self.running = False
        # self.cam.stop_recording(save_to_filename='genesis_video.mp4', fps=60)
        if self.keyboard_controller:
            self.keyboard_controller.stop()
        if self.sim_thread:
            self.sim_thread.join(timeout=1.0)  # 等待仿真线程结束

        self.socket.close()
        self.context.term()
        print("Socket is exiting...")
        print("Simulator stopped function exiting...")