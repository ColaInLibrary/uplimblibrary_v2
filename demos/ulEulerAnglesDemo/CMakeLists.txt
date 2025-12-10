find_package(Boost REQUIRED)

add_executable(
        ulEulerAnglesDemo
        ulEulerAnglesDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulEulerAnglesDemo
        math
        mdl
        std
        Boost::headers
)
