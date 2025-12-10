find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

set(
	HDRS
		Body.h
		CartesianMotion.h
		Controller.h
		JointsMotion.h
		JointsSecurity.h
#		MotionAdapter.h
#		MotionStrategy.h
#		ControllerWrapper.h
		RobotFactory.h
		Teach.h
		pid_msd.h
		InteractionControl.h
)

set(
	SRCS
		Body.cpp
		CartesianMotion.cpp
		Controller.cpp
#		ControllerWrapper.cpp
		JointsMotion.cpp
		JointsSecurity.cpp
		RobotFactory.cpp
		Teach.cpp
		pid_msd.cpp
		InteractionControl.cpp
)


add_library(
	controller
	${HDRS}
	${SRCS}
#	${ul_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(controller PUBLIC cxx_std_17)
endif()

target_include_directories(
	controller
	PUBLIC
	$<BUILD_INTERFACE:${ul_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${ul_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
)

target_link_libraries(
	controller
		math
		mdl
		std
		Boost::headers
)

target_link_libraries(controller   yaml-cpp::yaml-cpp)



set_target_properties(
	controller
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	OUTPUT_NAME ulcontroller
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
	VISIBILITY_INLINES_HIDDEN ON
)


install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/controller COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS controller
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
	install(
		TARGETS controller
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
	if(BUILD_SHARED_LIBS)
		install(
			TARGETS controller
			EXPORT ul
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
		)
	endif()
endif()


#generate_export_header(
#	controller
#	EXPORT_FILE_NAME export.h
#	PREFIX_NAME UL_
#)

#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/controller COMPONENT development)
