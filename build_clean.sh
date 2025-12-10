#!/bin/bash
# 清除编译内容的脚本

BUILD_DIR="build"

# 检查目录是否存在
if [ -d "$BUILD_DIR" ]; then
  echo "正在清除编译内容：$BUILD_DIR..."
  rm -rf "$BUILD_DIR"
  echo "编译目录已清除"
else
  echo "编译目录不存在$BUILD_DIR"
fi

