#!/bin/bash
###
 # @Description:  
 # @Author: AN Hao, YUAN Hanqing
 # @Date: 2025-06-09 13:59:13
 # @LastEditors: YUAN Hanqing
 # @LastEditTime: 2025-07-04 15:26:07
 # @FilePath: /uplimlibrary/build.sh
### 
set -e  # 遇到错误立即终止脚本

# 创建 build 目录（如果不存在）
mkdir -p build

# 进入 build 目录并执行编译
cd build
cmake ..         # 生成构建系统
make -j$(nproc)  # 并行编译

echo "编译完成！" 


# # # 远程真机 (17自由度双臂平台)
scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.8.65:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.8.65:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/

# # # 远程真机 (17自由度双臂平台2)
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.8.65:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.8.65:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb

# 远程真机 (WA3样机)
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.9.216:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.9.216:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb

# # 远程真机 (WA2样机)
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.8.47:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.8.47:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb

# # # # 远程真机 (I3双臂平台)
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.9.221:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.9.221:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb

# # # 远程真机 (180样机)
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.10.157:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.10.157:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb


# # # 远程真机 (串联腰平台)
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/libuplimb_library.so* nav01@172.16.9.232:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/
# scp /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/src/ul/pack/UplimbController.h nav01@172.16.9.232:/home/nav01/CodeFiles/xenomaixddpproject/test_new_lib/lib/uplimb/


# # 仿真
# cp -r /home/nav01/Downloads/xenomai-stabe-v4/uplimlibrary/build/lib/ /home/nav01/Downloads/uplimb_interface/python_sim/

