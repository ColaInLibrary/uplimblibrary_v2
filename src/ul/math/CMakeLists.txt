find_package(Boost REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)

set(
        BASE_HDRS
        algorithm.h
        Constants.h
        FourierSeries.h
        Function.h
        Matrix.h
        MatrixBaseAddons.h
        Polynomial.h
        PolynomialQuaternion.h
        Pose2Transformation.h
        Quaternion.h
        QuaternionBaseAddons.h
        Real.h
        Rotation.h
        SingleSShapeVelocity.h
        Spatial.h
        Spline.h
        SplineQuaternion.h
        SShapeVelocity.h
        Transform.h
        TransformAddons.h
        TypeTraits.h
        Vector.h
)
list(APPEND HDRS ${BASE_HDRS})

set(
        SPATIAL_HDRS
        spatial/ArticulatedBodyInertia.h
        spatial/ArticulatedBodyInertia.hxx
        spatial/ForceVector.h
        spatial/ForceVector.hxx
        spatial/MotionVector.h
        spatial/MotionVector.hxx
        spatial/PlueckerTransform.h
        spatial/PlueckerTransform.hxx
        spatial/RigidBodyInertia.h
        spatial/RigidBodyInertia.hxx
)
list(APPEND HDRS ${SPATIAL_HDRS})

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
    add_library(math INTERFACE ${HDRS})
else()
    add_library(math INTERFACE)
    add_custom_target(math_sources SOURCES ${HDRS})
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    target_compile_definitions(math INTERFACE EIGEN_DONT_ALIGN_STATICALLY)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
    target_compile_features(math INTERFACE cxx_std_17)
endif()

target_include_directories(
        math
        INTERFACE
        $<BUILD_INTERFACE:${ul_BINARY_DIR}/src>
        $<BUILD_INTERFACE:${ul_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
)

target_link_libraries(
        math
        INTERFACE
        Boost::headers
        Eigen3::Eigen
)

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
    set_target_properties(
            math
            PROPERTIES
            VERSION ${PROJECT_VERSION}
    )
endif()

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/math COMPONENT development)
install(FILES ${SPATIAL_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/math/spatial COMPONENT development)

install(
        TARGETS math
        EXPORT ul
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
