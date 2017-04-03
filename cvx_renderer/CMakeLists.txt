cmake_minimum_required(VERSION 2.6.0)

project(cvx_renderer)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake/ ${CMAKE_MODULE_PATH} )

FIND_PACKAGE(Boost 1.49 REQUIRED COMPONENTS system timer filesystem regex serialization thread iostreams date_time chrono random program_options)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Assimp)
FIND_PACKAGE(PkgConfig REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLFW3 3.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cvx_util
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES
	cvx_renderer
)

include_directories(
	include
	${EIGEN3_INCLUDE_DIR}
	${GLFW3_INCLUDE_DIR}
  	${catkin_INCLUDE_DIRS}
)


IF ( CMAKE_COMPILER_IS_GNUCXX )
	SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wreturn-type" )
ENDIF( CMAKE_COMPILER_IS_GNUCXX )

SET(CVX_RENDERER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/cvx/renderer/)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)
ADD_SUBDIRECTORY(render_tray)


