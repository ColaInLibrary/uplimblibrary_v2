find_package(Boost REQUIRED)

add_executable(
        ulCubicSplineDemo
        ulCubicSplineDemo.cpp
        #        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulCubicSplineDemo
        math
        mdl
        std
        Boost::headers
)
