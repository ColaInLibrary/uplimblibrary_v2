#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2025/5/14 下午4:36
# @Author  : AN Hao
# @File    : keyboard_controller.py
# @Description :
from pynput import keyboard
from typing import Dict, Callable

class KeyboardController:
    """Handles keyboard input and task execution"""

    def __init__(self, task_mapping: Dict[str, Callable], quit_callback: Callable):
        """
        Initialize keyboard controller

        Args:
            task_mapping: Dictionary mapping keys to task functions
            quit_callback: Function to call when quitting
        """
        self.task_mapping = task_mapping
        self.quit_callback = quit_callback
        self.running = False
        self.listener = None

    def _on_press(self, key):
        """Handle key press events"""
        try:
            if key.char == 'q':
                print("\n'q' detected, preparing to quit...")
                self.quit_callback()  # 调用退出回调函数
                return False  # Stop listener

            if key.char in self.task_mapping:
                print(f"\n'{key.char}' detected, executing task {key.char}...")
                self.task_mapping[key.char]()

        except AttributeError:
            pass

    def start(self):
        """Start keyboard listener"""
        print("Keyboard listener started, Press:")
        print("  - 'q': quit")
        for key, func in self.task_mapping.items():
            print(f"  - '{key}': {func.__name__}")

        self.running = True
        with keyboard.Listener(on_press=self._on_press) as listener:
            self.listener = listener
            listener.join()

    def stop(self):
        """Stop keyboard listener"""
        if self.listener:
            self.listener.stop()