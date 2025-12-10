#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/5/14 下午4:37
# @Author  : AN Hao
# @File    : main.py
# @Description :
from mujoco_system import MujocoSimulator, KeyboardController, TaskDefinition
from mujoco_system.config import URDF_PATH, JOINT_NAMES

if __name__ == "__main__":
    # Create and start simulator
    simulator = MujocoSimulator(URDF_PATH, JOINT_NAMES)

    try:
        print("Starting simulator...")
        simulator.start()
    except KeyboardInterrupt:
        print("\nCtrl+C pressed, stopping simulator...")
        simulator.stop()
    except Exception as e:
        print(f"ERROR: {e}")
        simulator.stop()
    finally:
        print("Program exiting")