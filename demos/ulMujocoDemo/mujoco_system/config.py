#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/5/14 下午4:37
# @Author  : AN Hao
# @File    : config.py
# @Description :
# Configuration constants
import os
project_root = os.path.dirname(os.path.abspath(__file__))
# ul.init(os.path.join(project_root, "..", "config", "robot_define_upper_body.yaml"))

# GENESIS related models
# URDF_PATH = os.path.join(project_root, "..", "models", "URDF-H1_Pro", "urdf", "URDF-H1_Pro-UPPER_BODY_ROBOT.urdf")
URDF_PATH = "/home/ah/Documents/example_code/robot_models/URDF-H1_Pro_mujoco/urdf/scene.xml"
JOINT_NAMES = [
    'Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L', 'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L',
    'Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R', 'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R',
    'Neck_Z', 'Neck_Y', 'A_Waist'
]