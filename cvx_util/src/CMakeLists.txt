set ( LIB_CVX_UTIL_SOURCES

	misc/binary_stream.cpp
#	misc/application_settings.cpp
	misc/logger.cpp
	misc/dictionary.cpp
	misc/progress_stream.cpp
	misc/filesystem.cpp

	geometry/trimesh_topology.cpp
	3rdparty/triangle.c
	3rdparty/nanoflann.hpp
	geometry/triangulate.cpp
	geometry/polygon_scanner.cpp
	geometry/kdtree.cpp
	geometry/octree.cpp
	geometry/util.cpp

	camera/calibration.cpp
	camera/pose.cpp
	3rdparty/RPP/RPP.cpp
	3rdparty/RPP/Rpoly.cpp

	viz/gnuplot.cpp

	imgproc/rgbd.cpp
	imgproc/concomp.cpp
	imgproc/gabor.cpp

	pcl/align.cpp
	pcl/icp.cpp

	math/rng.cpp
)

set ( LIB_CVX_UTIL_HEADERS

	${CVX_UTIL_INCLUDE_DIR}/misc/binary_stream.hpp
#	${CVX_UTIL_INCLUDE_DIR}/misc/application_settings.hpp
	${CVX_UTIL_INCLUDE_DIR}/misc/logger.hpp
	${CVX_UTIL_INCLUDE_DIR}/misc/dictionary.hpp
	${CVX_UTIL_INCLUDE_DIR}/misc/progress_stream.hpp
	${CVX_UTIL_INCLUDE_DIR}/misc/filesystem.hpp

	${CVX_UTIL_INCLUDE_DIR}/geometry/point.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/point_list.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/line.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/line_fit.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/rectangle.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/trimesh_topology.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/polygon.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/triangle.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/triangulate.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/polygon_scanner.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/kdtree.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/octree.hpp
	${CVX_UTIL_INCLUDE_DIR}/geometry/util.hpp

	${CVX_UTIL_INCLUDE_DIR}/camera/camera.hpp
	${CVX_UTIL_INCLUDE_DIR}/camera/calibration.hpp
	${CVX_UTIL_INCLUDE_DIR}/camera/pose.hpp

	${CVX_UTIL_INCLUDE_DIR}/viz/gnuplot.hpp

	${CVX_UTIL_INCLUDE_DIR}/math/random.hpp
	${CVX_UTIL_INCLUDE_DIR}/math/solvers/bfgs.hpp
	${CVX_UTIL_INCLUDE_DIR}/math/solvers/lbfgs.hpp
	${CVX_UTIL_INCLUDE_DIR}/math/solvers/gradient_descent.hpp

	${CVX_UTIL_INCLUDE_DIR}/imgproc/rgbd.hpp
	${CVX_UTIL_INCLUDE_DIR}/imgproc/concomp.hpp
	${CVX_UTIL_INCLUDE_DIR}/imgproc/gabor.hpp

	${CVX_UTIL_INCLUDE_DIR}/pcl/align.hpp
	${CVX_UTIL_INCLUDE_DIR}/pcl/icp.hpp

	${CVX_UTIL_INCLUDE_DIR}/math/rng.hpp

)

set_source_files_properties(rgbd/depth_filter/jointNearest.cpp
							rgbd/depth_filter/split.cpp
							PROPERTIES COMPILE_FLAGS -msse4.1)

#linking 

add_subdirectory(3rdparty/apriltag)

add_library(cvx_util SHARED ${LIB_CVX_UTIL_SOURCES} ${LIB_CVX_UTIL_HEADERS})

TARGET_LINK_LIBRARIES(cvx_util
	apriltag
	${Boost_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${catkin_LIBRARIES}
)



