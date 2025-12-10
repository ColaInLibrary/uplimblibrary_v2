find_package(Boost REQUIRED)

add_executable(
        ulIKDemo
        ulIKDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulIKDemo
        math
        mdl
        std
        Boost::headers
)
