find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

add_executable(
        ulMoveJDemo
        ulMoveJDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulMoveJDemo
        controller
        hal
        math
        mdl
        std
        rbt
        Boost::headers
        Threads::Threads
)
