find_package(Boost REQUIRED)

add_executable(
        ulCoachDemo
        ulCoachDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulCoachDemo
        controller
        hal
        math
        mdl
        std
        rbt
        Boost::headers
)
