find_package(Boost REQUIRED)

add_executable(
        ulQuaternionDemo
        ulQuaternionDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulQuaternionDemo
        math
        mdl
        std
        Boost::headers
)
