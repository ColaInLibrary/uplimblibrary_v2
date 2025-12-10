find_package(Boost REQUIRED)

set(
	HDRS
		algorithm.h
		common.h
		errorCode.h
		DoubleBuffer.h
		fileSave.h
		iterator.h
		GaussianFilter.h
		Differentiator.h
		ButterworthFilter.h
		DebugShareBuffer.h
		Socket.h
)

set(
	SRCS
		GaussianFilter.cpp
		Differentiator.cpp
		ButterworthFilter.cpp
		DebugShareBuffer.cpp
)

add_library(
	std
	${HDRS}
	${SRCS}
#	${ul_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(std PUBLIC cxx_std_17)
endif()

# 手动查找 ZeroMQ 库
find_library(ZMQ_LIBRARY
		NAMES zmq libzmq
		PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

# 查找头文件
find_path(ZMQ_INCLUDE_DIR
		NAMES zmq.h
		PATHS /usr/include /usr/local/include
)

target_include_directories(
	std
	PUBLIC
	$<BUILD_INTERFACE:${ul_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${ul_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
		${PINOCCHIO_INCLUDE_DIRS}
		${ZMQ_INCLUDE_DIR}
)

target_link_libraries(
	std
		math
		Boost::headers
		${ZMQ_LIBRARY}
)

set_target_properties(
	std
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	OUTPUT_NAME ulstd
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
	VISIBILITY_INLINES_HIDDEN ON
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/std COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS std
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
	install(
		TARGETS std
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
	if(BUILD_SHARED_LIBS)
		install(
			TARGETS std
			EXPORT ul
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
		)
	endif()
endif()


generate_export_header(
	std
	EXPORT_FILE_NAME export.h
	PREFIX_NAME UL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/std COMPONENT development)
