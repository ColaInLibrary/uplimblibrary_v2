cmake_minimum_required(VERSION 3.10...3.24)

if(POLICY CMP0072)
	cmake_policy(SET CMP0072 NEW)
endif()

if(POLICY CMP0075)
	cmake_policy(SET CMP0075 NEW)
endif()

#if(POLICY CMP0167)
#cmake_policy(SET CMP0167 NEW)
#endif()

project(ul VERSION 2.0.0)

if(NOT PROJECT_VERSION_TWEAK)
	set(PROJECT_VERSION_TWEAK 0)
endif()

option(REAL_MODE "Build for real-wrold robot environment" OFF)
option(UL_BUILD_CONTROLLER "Build controller" ON)
option(UL_BUILD_DEMOS "Build demos" ON)
option(UL_BUILD_HAL "Build Hardware" ON)
option(UL_BUILD_MATH "Build mathematics component" ON)
option(UL_BUILD_MDL "Build Models component" ON)
option(UL_BUILD_PACK "Build Package component" ON)
option(UL_BUILD_RBT "Build Robot Type" ON)
option(UL_BUILD_TESTS "Build tests" ON)

if(REAL_MODE)
    message(STATUS "Building for real hardware")
    execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=posix --skin=alchemy --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
	execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=posix --skin=alchemy --auto-init-solib --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

set(Boost_ADDITIONAL_VERSIONS "1.73.0" "1.73" "1.72.0" "1.72" "1.71.0" "1.71" "1.70.0" "1.70" "1.69.0" "1.69" "1.68.0" "1.68" "1.67.0" "1.67" "1.66.0" "1.66" "1.65.1" "1.65.0" "1.65" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT system)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
if(REAL_MODE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS} -Wl,-rpath,/usr/xenomai/lib")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 ")
# set(CMAKE_DEBUG_TYPE Debug)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 ")

include(CheckCXXCompilerFlag)
include(CMakeDependentOption)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)


#cmake_dependent_option(UL_BUILD_HAL "Build hardware abstraction layer component" ON "UL_BUILD_MATH;UL_BUILD_UTIL" OFF)
#cmake_dependent_option(UL_BUILD_MDL "Build rigid body kinematics and dynamics component" ON "UL_BUILD_MATH;UL_BUILD_XML" OFF)

#cmake_dependent_option(UL_BUILD_PLAN "Build path planning component" ON "UL_BUILD_KIN;UL_BUILD_MATH;UL_BUILD_MDL;UL_BUILD_SG;UL_BUILD_UTIL;UL_BUILD_XML" OFF)

add_subdirectory(src)

if(UL_BUILD_DEMOS)
	message(STATUS "UL_BUILD_DEMOS = ${UL_BUILD_DEMOS}")
	add_subdirectory(demos)
endif()

foreach(target IN LISTS TARGETS)
	list(APPEND UL_LIBRARIES "ul::${target}")
	get_target_property(TARGET_INTERFACE_LINK_LIBRARIES ${target} INTERFACE_LINK_LIBRARIES)
	if(TARGET_INTERFACE_LINK_LIBRARIES)
		foreach(library IN LISTS TARGET_INTERFACE_LINK_LIBRARIES)
			if(library MATCHES "([A-Za-z0-9_.+-]*)::([A-Za-z0-9_.+-]*)")
				list(APPEND PACKAGES ${CMAKE_MATCH_1})
			endif()
		endforeach()
	endif()
endforeach()

list(REMOVE_DUPLICATES PACKAGES)
list(SORT PACKAGES)

foreach(package IN LISTS PACKAGES)
	list(APPEND FIND_DEPENDENCIES "find_dependency(${package})")
	list(APPEND MODULES ${package})
	message(STATUS "${package}")
	if(package STREQUAL "fcl")
		message(STATUS "Adding dependencies for fcl: Boost, ccd, Eigen3, octomap")
		list(APPEND MODULES Boost ccd Eigen3 octomap)
	elseif(package STREQUAL "LibXml2")
		message(STATUS "Adding dependencies for LibXml2")
		list(APPEND MODULES Iconv ICU LibLZMA Threads ZLIB)
	elseif(package STREQUAL "LibXslt")
		message(STATUS "Adding dependencies for LibXslt")
		list(APPEND MODULES LibXml2)
	elseif(package STREQUAL "ODE")
		message(STATUS "Adding dependencies for ODE")
		list(APPEND MODULES ccd Threads)
	endif()
endforeach()


list(REMOVE_DUPLICATES MODULES)
list(SORT MODULES)

foreach(module IN LISTS MODULES)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Find${module}.cmake)
		file(
			COPY cmake/Find${module}.cmake
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib/cmake/ul
		)
		install(
			FILES cmake/Find${module}.cmake
			DESTINATION ${CMAKE_SOURCE_DIR}/cmake/ul-${PROJECT_VERSION}
			COMPONENT development
		)
	endif()
endforeach()

string(REPLACE ";" "\n" FIND_DEPENDENCIES "${FIND_DEPENDENCIES}")

export(
	TARGETS ${TARGETS}
	NAMESPACE ul::
	FILE ${CMAKE_CURRENT_BINARY_DIR}/lib/cmake/ul/ul-export.cmake
)

configure_package_config_file(
	ul-config.cmake.in lib/cmake/ul/ul-config.cmake
	INSTALL_DESTINATION ${CMAKE_SOURCE_DIR}/cmake/ul-${PROJECT_VERSION}
)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/cmake/ul/ul-config.cmake
	DESTINATION ${CMAKE_SOURCE_DIR}/cmake/ul-${PROJECT_VERSION}
	COMPONENT development
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/lib/cmake/ul/ul-config-version.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY ExactVersion
)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/cmake/ul/ul-config-version.cmake
	DESTINATION ${CMAKE_SOURCE_DIR}/cmake/ul-${PROJECT_VERSION}
	COMPONENT development
)

if(DEFINED TARGETS)
	install(
		EXPORT ul
		DESTINATION ${CMAKE_SOURCE_DIR}/cmake/ul-${PROJECT_VERSION}
		NAMESPACE ul::
		FILE ul-export.cmake
		COMPONENT development
	)
endif()
message(STATUS "PROJECT_VERSION = ${PROJECT_VERSION}")

#include(CPackConfig)
#include(CPack)

#################################################################################
####################### 编译生成库文件和复制头文件 ###################################
#################################################################################
# 假设所有源文件位于src目录下
file(GLOB_RECURSE SRC_FILES "src/*.cpp")
file(GLOB_RECURSE SRC_HEADERS "src/*.h")

# 添加共享库
if(REAL_MODE)
	set(TARGET_NAME uplimb_library)
else()
	set(TARGET_NAME uplimb_library)
endif()

add_library(${TARGET_NAME} SHARED ${SRC_FILES})

# 定义一个选项，允许外部项目设置 CONFIG_FILE_PATH
set(CONFIG_FILE_PATH "" CACHE STRING "Path to config file")

# 如果定义了 CONFIG_FILE_PATH，则添加宏定义
if(CONFIG_FILE_PATH)
    target_compile_definitions(${TARGET_NAME} PRIVATE CONFIG_FILE_PATH="${CONFIG_FILE_PATH}")
endif()

if(REAL_MODE)
	# 定义宏 REAL_MODE=1，代码中可以用 #if REAL_MODE
	target_compile_definitions(${TARGET_NAME} PRIVATE REAL_MODE=1)
endif()
# 设置包含目录，这里使用 target_include_directories 更加明确
target_include_directories(${TARGET_NAME}
		PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src> # 对于构建时
		$<INSTALL_INTERFACE:include> # 对于安装时
		"/opt/openrobots/include"
)
target_include_directories(${TARGET_NAME}
		PUBLIC
		$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/ul-${PROJECT_VERSION}>
		${PINOCCHIO_INCLUDE_DIRS}
)
message(STATUS "Pinocchio include dirs: ${PINOCCHIO_INCLUDE_DIRS}")
# 设置目标属性（如需）
set_target_properties(${TARGET_NAME} PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION 1
)


find_package(Boost REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
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

find_package(yaml-cpp REQUIRED)

# 如果有其他库需要链接进来，可以在此处添加
target_link_libraries(${TARGET_NAME}
		# 在这里列出需要链接的库，例如：
		Boost::headers
		Eigen3::Eigen
		${PINOCCHIO_LIBRARIES}
)
target_link_libraries(${TARGET_NAME}   yaml-cpp::yaml-cpp)


# 在这里添加 install 命令以指定 ${TARGET_NAME} 库的安装位置
install(TARGETS ${TARGET_NAME}
		EXPORT ul-shared-export
		LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib  # 直接指定到工程根目录下的 install/lib 文件夹
#		ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib  # 如果有的话，这是静态库的安装目录
		RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin  # 如果有的话，这是可执行文件的安装目录
)

## 安装 src/pack 目录下的头文件到工程根目录下的 install/include 文件夹
install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ul/pack/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include  # 目标安装路径
		FILES_MATCHING PATTERN "*.h"  # 只匹配 .h 结尾的文件
		PATTERN ".git" EXCLUDE  # 排除不必要的目录（如果有的话）
)