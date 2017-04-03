cmake_minimum_required(VERSION 2.8.0)

project(cvx_orec_linemod)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake/ ${CMAKE_MODULE_PATH} )

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PCL)
find_package(Boost 1.49 REQUIRED COMPONENTS system filesystem chrono program_options)
find_package(OpenCV 3 REQUIRED COMPONENTS core highgui video imgproc rgbd)
find_package(Eigen3 REQUIRED)
find_package(Assimp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cvx_util
	cvx_renderer
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES
	cvx_orec_linemod
)

include_directories(
        include
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${ASSIMP_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

IF ( CMAKE_COMPILER_IS_GNUCXX )
	SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wreturn-type" )
ENDIF( CMAKE_COMPILER_IS_GNUCXX )

SET(CVX_OREC_LINEMOD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/cvx/orec/linemod/)

# add subprojects
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)

