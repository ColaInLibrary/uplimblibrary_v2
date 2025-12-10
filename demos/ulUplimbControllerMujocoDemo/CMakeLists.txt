find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

# Mujoco 仿真
set(MUJOCO_DIR "/home/ah/Documents/github_example/mujoco-3.3.2-linux-x86_64/mujoco-3.3.2")
# 添加MuJoCo包含目录，这样编译器能找到对应的头文件
include_directories(${MUJOCO_DIR}/include)
include_directories(${MUJOCO_DIR}/include/mujoco)
include_directories(/usr/include/GLFW)
find_package(glfw3 REQUIRED)
include_directories(${glfw3_INCLUDE_DIRS})
message(STATUS "Found GLFW library: ${glfw3_LIBRARY}")
find_library(MUJOCO_LIBRARY NAMES
        libmujoco.so
        libmujoco.so.3.3.2
        PATHS ${MUJOCO_DIR}/lib NO_DEFAULT_PATH)
message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARY}")


find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq) # 查找 ZMQ 库

include_directories(${ZMQ_INCLUDE_DIRS} ${MUJOCO_DIR}/include/mujoco include)
# 源文件
file(GLOB SOURCES "src/*.cpp")

add_executable(
        ulUplimbControllerMujoco
        ${SOURCES}
        ulUplimbControllerMujoco.cpp
)

#target_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
        ulUplimbControllerMujoco
        controller
        hal
        math
        mdl
        pack
        std
        Boost::headers
        Threads::Threads
        ${MUJOCO_LIBRARY}
        glfw
        ${ZMQ_LIBRARIES}
)
