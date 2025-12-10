find_package(Boost REQUIRED)

# 设置 CMAKE_PREFIX_PATH 来添加 Pinocchio 安装目录
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/openrobots")

find_package(pinocchio REQUIRED)
# 检查是否找到了 Pinocchio
if(pinocchio_FOUND)
	# 包含 Pinocchio 的头文件路径
	include_directories(${PINOCCHIO_INCLUDE_DIRS})
	message(STATUS "pinochhio = ${PINOCCHIO_LIBRARIES}")
	message(STATUS "pinochhio = ${PINOCCHIO_INCLUDE_DIRS}")
endif()


set(
	HDRS
		AnalyticalInverseKinematics.h
		Dynamic.h
		Kinematic.h
		Model.h
)

set(
	SRCS
		AnalyticalInverseKinematics.cpp
		Dynamic.cpp
		Kinematic.cpp
		Model.cpp
)


add_library(
	mdl
	${HDRS}
	${SRCS}
#	${ul_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(mdl PUBLIC cxx_std_17)
endif()

target_include_directories(
	mdl
	PUBLIC
	$<BUILD_INTERFACE:${ul_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${ul_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
		${PINOCCHIO_INCLUDE_DIRS}
)

target_link_libraries(
	mdl
		math
		std
		Boost::headers
		${PINOCCHIO_LIBRARIES}
)

set_target_properties(
	mdl
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	OUTPUT_NAME ulmdl
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
	VISIBILITY_INLINES_HIDDEN ON
)


install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/mdl COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS mdl
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
	install(
		TARGETS mdl
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
	if(BUILD_SHARED_LIBS)
		install(
			TARGETS mdl
			EXPORT ul
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
		)
	endif()
endif()


generate_export_header(
	mdl
	EXPORT_FILE_NAME export.h
	PREFIX_NAME UL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/mdl COMPONENT development)
