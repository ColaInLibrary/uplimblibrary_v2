find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

# raisim 仿真
find_package(raisim 1.1.7 REQUIRED)
include_directories(${raisim_INCLUDE_DIRS})
message(STATUS "raisim_DIRS = ${raisim_INCLUDE_DIRS}")
link_directories(${raisim_LIBRARY_DIRS})
message(STATUS "raisim_DIRS = ${raisim_LIBRARIES}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq) # 查找 ZMQ 库

add_executable(
        ulUplimbControllerRaisimDemo
        plot.h
        ulUplimbControllerRaisimDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

#target_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${ZMQ_INCLUDE_DIRS})

target_link_libraries(
        ulUplimbControllerRaisimDemo
        controller
        hal
        math
        mdl
        pack
        std
        Boost::headers
        Threads::Threads
        ${raisim_LIBRARIES}
        ${ZMQ_LIBRARIES}
)
