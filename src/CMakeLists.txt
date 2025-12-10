configure_file(ul/version.h.in ul/version.h)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ul/version.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul COMPONENT development)

add_subdirectory(ul/std)
list(APPEND TARGETS std)

if(UL_BUILD_HAL)
    add_subdirectory(ul/hal)
    list(APPEND TARGETS hal)
endif ()

if(UL_BUILD_MATH)
    add_subdirectory(ul/math)
    list(APPEND TARGETS math)
endif()

if(UL_BUILD_MDL)
    add_subdirectory(ul/mdl)
    list(APPEND TARGETS mdl)
endif()

#if(UL_BUILD_RBT)
#    add_subdirectory(ul/rbt)
#    list(APPEND TARGETS rbt)
#endif()

if(UL_BUILD_CONTROLLER)
    add_subdirectory(ul/controller)
    list(APPEND TARGETS controller)
endif()

if(UL_BUILD_PACK)
    add_subdirectory(ul/pack)
    list(APPEND TARGETS pack)
endif()

set(TARGETS ${TARGETS} PARENT_SCOPE)
