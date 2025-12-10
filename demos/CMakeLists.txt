if(UL_BUILD_MATH)
#    add_subdirectory(ulEulerAnglesDemo)
#    add_subdirectory(ulModifiedDenavitHartenbergDemo)
    add_subdirectory(ulCubicSplineDemo)
    add_subdirectory(ulQuaternionDemo)
    add_subdirectory(ulSShapeVelocityDemo)
endif()

if (UL_BUILD_MDL)
    add_subdirectory(ulIKDemo)
endif()

if(UL_BUILD_RBT)
    add_subdirectory(ulForwardKinematicsDemo)
endif()

if(UL_BUILD_HAL)
#    add_subdirectory(ulCoachDemo)
endif()

if(UL_BUILD_PACK)
    # add_subdirectory(ulUplimbControllerRaisimDemo)
    add_subdirectory(ulUplimbControllerMujocoDemo)
endif ()

set(CPACK_NSIS_CREATE_ICONS_EXTRA ${CPACK_NSIS_CREATE_ICONS_EXTRA} PARENT_SCOPE)
set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} PARENT_SCOPE)
set(WIX_SHORTCUTS ${WIX_SHORTCUTS} PARENT_SCOPE)

#if(UNIX)
#    install(FILES ${UL_SOURCE_DIR}/robotics-library.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps COMPONENT demos)
#endif()
