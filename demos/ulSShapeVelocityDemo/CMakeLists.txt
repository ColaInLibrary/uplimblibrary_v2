find_package(Boost REQUIRED)

add_executable(
        ulSShapeVelocityDemo
        ulSShapeVelocityDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulSShapeVelocityDemo
        controller
        math
        mdl
        std
        Boost::headers
)
