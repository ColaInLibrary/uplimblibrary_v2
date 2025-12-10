#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/5/14 下午4:35
# @Author  : AN Hao
# @File    : task_executor.py
# @Description :
import time

import numpy as np
from .task_definition import TaskDefinition
from typing import List, Dict, Callable, Any
import pandas as pd
import os

def csv_to_numpy(file_path):
    """读取 CSV 文件并转换为 NumPy 数组"""
    try:
        df = pd.read_csv(file_path)
        numpy_array = df.to_numpy()  # 转换为 NumPy 数组
        print("NumPy 数组:")
        print(numpy_array)
        return numpy_array
    except Exception as e:
        print(f"读取文件时发生错误: {e}")
        return None


class TaskExecutor:
    """Handles execution of all robot tasks"""

    def __init__(self, simulator: Any, socket, dof: int, dofs_idx: List[int]):
        """
        Initialize task executor

        Args:
            simulator: Reference to the simulator object
            dof: Degrees of freedom
            dofs_idx: List of joint indices
        """
        self.simulator = simulator
        self.dof = dof
        self.dofs_idx = dofs_idx
        self.task_def = TaskDefinition(socket)

    def go_zero(self):
        """Reset all joints to zero position"""
        zero_positions = np.zeros(self.dof)
        self.task_def.moveJ(zero_positions.tolist(), 1.0, 1.0, False, 1)
        print("Task 'go_zero' executed.")

    def go_home(self):
        """Set to home position"""
        target_q = np.array([-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
                             -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15
                             ])
        self.task_def.moveJ(target_q.tolist(), 1.0, 1.0, False, 0x03)
        print("Task 'go_home' executed.")

    def moveJ_example(self):
        """Set to home position"""
        home_positions = np.array([
            -0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
            -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
            0, 0,
            0
        ])
        self.task_def.moveJ(home_positions.tolist())

        left_positions = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ])
        self.task_def.moveJ(left_positions.tolist(), arm_type=0x01)

        left_positions = np.array([
            0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0
        ])
        self.task_def.moveJ(left_positions.tolist(), arm_type=0x02)

        left_positions = np.array([0.5, -0.1])
        self.task_def.moveJ(left_positions.tolist(), arm_type=0x04)

        print("Task 'moveJ_example' executed.")

    # def moveJ_path_example(self):
    #     """Set to test pose"""
    #     print(">>>>> task_two")
    #     # path = np.array([[-0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #     #                   -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
    #     #                   0, 0, 0],
    #     #                  [-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #     #                   -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15,
    #     #                   0, 0, 0],
    #     #                  [-0.035,  0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #     #                   -0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038,
    #     #                   0, 0, 0]
    #     #                  ])
    #     #
    #     # ul.moveJ_path(path, 2, False, arm_type=-1)
    #     #
    #     # print("===============================")
    #     # path = np.array([[-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2],
    #     #                  [-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2],
    #     #                  [-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2]
    #     #                  ])
    #     # ul.moveJ_path(path, 2, False, arm_type=0)
    #     #
    #     print("===============================")
    #     start_time = time.time()
    #     path = np.array([[-0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038],
    #                      [-0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15],
    #                      [-0.223363, -0.38122, 0.189163, -0.819702, 0.566464, -0.0839337, 0.0170038]
    #                      ])
    #     ul.moveJ_path_uniform(path, time=2, arm_type=0x02)
    #     end_time = time.time()
    #     print(f"Time taken: {end_time - start_time} seconds")
    # 
    #     # print("===============================")
    #     # path = np.array([[0.0, -0.1],
    #     #                  [0.0, 0.2],
    #     #                  [0.0, 0.0]
    #     #                  ])
    #     # ul.moveJ_path_uniform(path, 2, False, arm_type=2)
    # 
    #     # print("===============================")
    #     # path = np.array([[0.0], [0.5], [0.0]])
    #     # ul.moveJ_path(path, 2, False, arm_type=3)
    # 
    #     print("Task 'moveJ_path_example' executed.")
    # 
    # def speedJ_example(self):
    #     """speedJ example"""
    #     target_qd = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                           0.3, 0.0, 0])
    #     ul.speedJ(target_qd, 0.5, 1.0)
    # 
    #     target_qd = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     ul.speedJ(target_qd, 0.5, 2.0, arm_type=0x02)
    #     ul.speedStop()
    # 
    #     target_qd = np.array([0.2, -0.0])
    #     ul.speedJ(target_qd, 0.5, 2.0, arm_type=0x04)
    #     ul.speedStop()
    # 
    #     target_qd = np.array([0.2])
    #     ul.speedJ(target_qd, 0.5, 2.0, arm_type=0x08)
    #     ul.speedStop()
    # 
    #     print("Task 'speedJ_example' executed.")
    # 
    # 
    # def speedStop_example(self):
    #     """speedStop example"""
    #     ul.speedStop()
    #     print("Task 'speedStop_example' executed.")
    # 
    # def moveL_example(self):
    #     """moveL example"""
    #     init_q = np.array([-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #                          -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15,
    #                          0, 0, 0])
    #     ul.moveJ(init_q, 2, 10)
    #     ee_pose = ul.getCurrentForwardKinematics()  # list[np.array, np.array]
    #     print(ee_pose)
    # 
    #     trg_q = np.array([-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #                       -1.06869,    0.0408411,     0.197856,    -0.963425,   -0.0395368,     0.664458, -1.14583e-16,
    #                       0, 0, 0])
    #     ee_pose = ul.getForwardKinematics((trg_q))
    #     ul.moveL(ee_pose[1], 0.15, 2, arm_type=0x02)  # Move to the target pose with linear interpolation
    # 
    #     # ee_pose2 = [arr.copy() for arr in ee_pose]  # numpy的匀速深度拷贝
    #     # ee_pose2[0][0] += 0.1
    #     # ee_pose2[0][1] += 0.15
    #     # ee_pose2[0][2] += 0.3
    #     # # ee_pose2[0][2] += 0.05
    #     # # ee_pose2[0][3] += 0.3
    #     # # ee_pose2[1][2] += 0.05
    #     # # ee_pose2[1][3] += 0.3
    #     # print(ee_pose[0])
    #     # print(ee_pose2[0])
    #     # for i in range(1):
    #     #     print(f" LOOP: {i} ")
    #     #     ul.moveL(ee_pose2, 1, 5)
    #     #     # time.sleep(1)
    #     #     # ee_pose2[1][2] -= 0.05
    #     #     # ul.moveL(ee_pose2[1], 0.25, 0.8, arm_type=1)
    #     #     # time.sleep(1)
    #     #     # ee_pose2[0][2] += 0.05
    #     #     # ul.moveL(ee_pose2[0], 0.25, 0.8, arm_type=0)
    # 
    # 
    #     # path = np.array([
    #     #   [0.38903604, -0.30935242,  0.17104938,  1.90521488, -1.47887397, -1.99480672],
    #     #   [0.38999839, -0.30808249,  0.16170752,  1.97930976, -1.46556088, -2.06553883],
    #     #   [0.39030678, -0.308427,    0.158791,    1.90232503, -1.47771524, -1.99169754],
    #     #   [0.39315174, -0.30811915,  0.15417429,  1.82091937, -1.48026492, -1.91685577],
    #     #   [0.39388516, -0.31152227,  0.14974167,  2.05652995, -1.46479571, -2.11758258],
    #     #   [0.39486027, -0.31068297,  0.14636898,  1.98031843, -1.45702271, -2.03858513],
    #     #   [0.4010894,  -0.31910656,  0.14236526,  1.98119951, -1.48036218, -1.96372399],
    #     #   [0.40056229, -0.31831005,  0.1370883,   2.11459541, -1.46940916, -2.10465159],
    #     #   [0.40431381, -0.32369173,  0.13024209,  2.24377129, -1.45678586, -2.17951256],
    #     #   [0.40492251, -0.32219126,  0.12684182,  2.0086582,  -1.48716356, -1.96295559],
    #     #   [0.40246979, -0.31898494,  0.12087692,  2.1635979,  -1.46282349, -2.13748476],
    #     #   [0.40506362, -0.32257035,  0.12348897,  2.19510856, -1.45998263, -2.13723338],
    #     #   [0.40503515, -0.32591086,  0.13398327,  2.29662187, -1.46967566, -2.21119876],
    #     #   [0.40469488, -0.32742408,  0.13936959,  2.36753249, -1.45003088, -2.26008026],
    #     #   [0.3997794,  -0.3205052,   0.15188391,  2.11103464, -1.47635293, -2.07709228],
    #     #   [0.39231544, -0.31180064,  0.16057414,  2.07207072, -1.47322969, -2.11917376],
    #     #   [0.38958972, -0.30857554,  0.16924027,  1.91328647, -1.45510839, -1.97505958],
    #     #   [0.38603752, -0.30753148,  0.17800959,  1.94754664, -1.45584227, -2.01649219],
    #     #   [0.38160461, -0.30739536,  0.17947433,  2.2420264,  -1.42807021, -2.31834297],
    #     #   [0.37881875, -0.30400638,  0.19135036,  2.27906078, -1.45049632, -2.38456751],
    #     #   [0.45373414, -0.07102384,  0.31603847,  1.25086828, -1.12752024, -0.7035869 ],
    #     #   [0.45373789, -0.06944051,  0.31588126,  1.23895452, -1.12395278, -0.69526068],
    #     #   [0.45118142, -0.06707263,  0.31752084,  1.2273271,  -1.11241318, -0.70525528],
    #     #   [0.45119398, -0.06008463,  0.30881124,  1.18185972, -1.10302371, -0.65527209],
    #     #   [0.45352694, -0.06216935,  0.29491713,  1.1486828,  -1.05389761, -0.62385213],
    #     #   [0.46547987, -0.06920672,  0.28548761,  1.0765038,  -1.05194631, -0.46544709],
    #     #   [0.465014,   -0.06912575,  0.26986335,  1.14225848, -1.04437314, -0.55549913],
    #     #   [0.45864471, -0.07001144,  0.25983927,  1.24225913, -1.03106991, -0.62043826],
    #     #   [0.45703112, -0.07286179,  0.25049023,  1.2578337,  -1.00072721, -0.64037425],
    #     #   [0.4706771,  -0.06961409,  0.24581937,  1.20671003, -0.93742449, -0.47395536],
    #     #   [0.47571874, -0.05173736,  0.2468045,   1.06993242, -0.85945422, -0.45495231],
    #     #   [0.47420927, -0.04542226,  0.24421856,  1.17530475, -0.92976981, -0.5650466 ],
    #     #   [0.48103333, -0.03824537,  0.24270128,  1.1580657,  -0.97751155, -0.53136209],
    #     #   [0.4866819,  -0.03051578,  0.24390859,  1.17558355, -0.96099439, -0.5411486 ],
    #     #   [0.47561058, -0.0175448,   0.24438475,  1.19429264, -0.97835131, -0.66183616],
    #     #   [0.46353073, -0.02000932,  0.24515443,  1.26467014, -1.05778023, -0.80428338],
    #     #   [0.45795204, -0.02740428,  0.24634378,  1.33456081, -1.12447057, -0.91835186],
    #     #   [0.45942605, -0.02184786,  0.26426732,  1.2076797,  -1.12190553, -0.823893  ],
    #     #   [0.46451559, -0.02358089,  0.28364787,  1.16680917, -1.12260982, -0.79372614],
    #     #   [0.46906487, -0.0305704,   0.29026702,  1.15489736, -1.15567176, -0.72661238]])
    #     # path_time = 40*0.25  # 每个点的时间间隔为0.25秒
    #     # ul.moveL_path_uniform(path, path_time, False, arm_type=1)
    #     print("Task 'moveL_example' executed.")
    # 
    # def speedL_example(self):
    #     """speedL example"""
    #     target_q = np.array([-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #                          -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15,
    #                          0, 0, 0])
    #     ul.moveJ(target_q, 2, 10)
    #     target_xd = [np.array([0.0, 0.0, 0.03, 0.0, 0.0, 0.0]),
    #                  np.array([0.0, 0.0, 0.03, 0.0, 0.0, 0.0])]
    # 
    #     ul.speedL(target_xd, 0.5, 0.0)
    #     time.sleep(2)
    #     ul.speedStop()
    #     time.sleep(2)
    #     ul.speedL(target_xd[0], 0.5, 0.0, arm_type=0x01)
    #     time.sleep(2)
    #     ul.speedStop()
    #     time.sleep(2)
    #     print("Task 'speedL_example' executed.")
    # 
    # def servoJ_example(self):
    #     """servoJ example"""
    #     # 读取CSV文件
    #     project_root = os.path.dirname(os.path.abspath(__file__))
    #     df = pd.read_csv(os.path.join(project_root, '..', 'data', 'filterNewSession-008_100hz_fragment.csv'))  # 替换为你的CSV文件路径
    # 
    #     # 定义left_q和neck_q
    #     left_q = np.array([0, 0.2, 0, -0.5, 0, 0, 0])
    #     neck_q = np.array([0, 0])
    #     waist_q = np.array([0])
    # 
    #     # 遍历每一行
    #     for index, row in df.iterrows():
    #         # 提取right_q（pos_1到pos_7）
    #         right_q = row[['pos_1', 'pos_2', 'pos_3', 'pos_4', 'pos_5', 'pos_6', 'pos_7']].values
    # 
    #         # 构建q数组
    #         q = np.concatenate([left_q, right_q, neck_q, waist_q])
    # 
    #         # 确保q的大小是17
    #         assert q.size == 17, f"q的大小不是17，而是{q.size}"
    # 
    #         # ul.servoJ(q, 0.0, 0.0, 0.01, 0.2, 100.0)
    #         ul.moveJ_with_freq(right_q, 50.0, asynchronous=True, arm_type=0x02)
    #         time.sleep(0.01)
    # 
    #     print("Task 'servoJ_example' executed.")
    # 
    # 
    # def moveJ_pose_example(self):
    #     """moveJ pose example"""
    #     target_q = np.array([-0.035, 0.533, -0.096, -1.348, -0.787, 0.008, 0.2,
    #                          -0.049, -0.508, -0.082, -1.302, 0.809, -0.01, -0.15,
    #                          0, 0, 0])
    #     ul.moveJ(target_q, 2, 10)
    #     q7 = np.array([target_q[6], target_q[13]])
    #     ee_pose = ul.getForwardKinematics(target_q)  # list[np.array, np.array]
    #     # print(ee_pose)
    #     ee_pose2 = [arr.copy() for arr in ee_pose]  # numpy的匀速深度拷贝
    #     ee_pose2[0][2] += 0.05
    #     ee_pose2[0][3] += 0.3
    #     ee_pose2[1][2] += 0.05
    #     ee_pose2[1][3] += 0.3
    # 
    #     ul.moveJ_pose(ee_pose2, q7)
    # 
    #     ee_pose2[1][2] -= 0.05
    #     ul.moveJ_pose(ee_pose2[1], q7[1], arm_type=0x02)
    # 
    #     print("Task 'moveJ_pose_example' executed.")
    # 
    # 
    # def smooth_example(self):
    #     """load Original CSV example"""
    #     ul.getCSVFile("/home/ah/Documents/python_code/h5_deal/shenquanpengquan_filtered_1000hz.csv")
    #     ul.CSVFileMove(False)
    # 
    # 
    # def toppra_example(self):
    #     """load Original CSV example"""
    #     ul.getCSVFile("/home/ah/Documents/github_example/toppra/optimized_trajectory.csv")
    #     ul.CSVFileMove(False)
