find_package(Boost REQUIRED)

add_executable(
        ulForwardKinematicsDemo
        ulForwardKinematicsDemo.cpp
        #        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulForwardKinematicsDemo
        math
        mdl
        std
        Boost::headers
)
