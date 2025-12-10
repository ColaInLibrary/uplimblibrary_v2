find_package(Boost REQUIRED)

add_executable(
        ulModifiedDenavitHartenbergDemo
        ulModifiedDenavitHartenbergDemo.cpp
#        ${ul_BINARY_DIR}/uplimb-library.rc
)

target_link_libraries(
        ulModifiedDenavitHartenbergDemo
        math
        mdl
        std
        Boost::headers
)
