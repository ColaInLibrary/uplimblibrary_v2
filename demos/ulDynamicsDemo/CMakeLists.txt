find_package(Boost REQUIRED)

add_executable(
        ulDynamicsDemo
        ulDynamicsDemo.cpp
        #        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulDynamicsDemo
        math
        mdl
        std
        rbt
        Boost::headers
)
