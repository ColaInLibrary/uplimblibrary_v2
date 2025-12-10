include(TestBigEndian)

find_package(Boost REQUIRED)

set(
	HDRS
		AxisController.h
		Coach.h
		CyclicDevice.h
		JointAccelerationActuator.h
		JointControlWord.h
		JointCurrentSensor.h
		JointMode.h
		JointModeOperation.h
		JointOffsetTorqueActuator.h
		JointOffsetVelocityActuator.h
		JointPositionActuator.h
		JointPositionSensor.h
		JointStatusWord.h
		JointTorqueActuator.h
		JointTorqueSensor.h
		JointVelocityActuator.h
		JointVelocitySensor.h
)

set(
	SRCS
		AxisController.cpp
		Coach.cpp
		CyclicDevice.cpp
		JointAccelerationActuator.cpp
		JointControlWord.cpp
		JointCurrentSensor.cpp
		JointMode.cpp
		JointModeOperation.cpp
		JointOffsetTorqueActuator.cpp
		JointOffsetVelocityActuator.cpp
		JointPositionActuator.cpp
		JointPositionSensor.cpp
		JointStatusWord.cpp
		JointTorqueActuator.cpp
		JointTorqueSensor.cpp
		JointVelocityActuator.cpp
		JointVelocitySensor.cpp
)

add_library(
	hal
	${HDRS}
	${SRCS}
#	${ul_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(hal PUBLIC cxx_std_17)
endif()

target_include_directories(
	hal
	PUBLIC
	$<BUILD_INTERFACE:${ul_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${ul_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
)

target_link_libraries(
	hal
	math
	util
	Boost::headers
)

set_target_properties(
	hal
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	OUTPUT_NAME ulhal
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
	VISIBILITY_INLINES_HIDDEN ON
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/hal COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS hal
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
	install(
		TARGETS hal
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
	if(BUILD_SHARED_LIBS)
		install(
			TARGETS hal
			EXPORT ul
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
		)
	endif()
endif()

generate_export_header(
	hal
	EXPORT_FILE_NAME export.h
	PREFIX_NAME UL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/hal COMPONENT development)
