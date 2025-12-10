find_package(Boost REQUIRED)

set(
	HDRS
		UplimbController.h
)

set(
	SRCS
		UplimbController.cpp
)


add_library(
	pack
	${HDRS}
	${SRCS}
#	${ul_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(pack PUBLIC cxx_std_17)
endif()

target_include_directories(
	pack
	PUBLIC
	$<BUILD_INTERFACE:${ul_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${ul_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
)

target_link_libraries(
	pack
		hal
		math
		mdl
		std
		controller
	Boost::headers
)

set_target_properties(
	pack
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	OUTPUT_NAME ulpack
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
	VISIBILITY_INLINES_HIDDEN ON
)


install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/pack COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS pack
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
	install(
		TARGETS pack
		EXPORT ul
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
	if(BUILD_SHARED_LIBS)
		install(
			TARGETS pack
			EXPORT ul
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
		)
	endif()
endif()


generate_export_header(
	pack
	EXPORT_FILE_NAME export.h
	PREFIX_NAME UL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}/ul/pack COMPONENT development)
