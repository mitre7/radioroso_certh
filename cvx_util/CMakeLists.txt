cmake_minimum_required(VERSION 2.8.0)

project(cvx_util)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake/ ${CMAKE_MODULE_PATH} )

find_package(OpenMP)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(Boost 1.49 REQUIRED COMPONENTS system filesystem regex serialization thread random)
FIND_PACKAGE(OpenCV 3 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES
	cvx_util
  DEPENDS
	Boost
	Eigen
	OpenCV
	PCL
)


include_directories(
	${PROJECT_SOURCE_DIR}/include/
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	src/3rdparty/
)

IF ( CMAKE_COMPILER_IS_GNUCXX )
	SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wreturn-type" )
ENDIF( CMAKE_COMPILER_IS_GNUCXX )

SET ( CVX_UTIL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/cvx/util/ )

ADD_SUBDIRECTORY(src)



