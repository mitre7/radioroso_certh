add_executable(test_linefit test_linefit.cpp)
target_link_libraries(test_linefit cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_triangulate test_triangulate.cpp)
target_link_libraries(test_triangulate cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_polyscanner test_polyscanner.cpp)
target_link_libraries(test_polyscanner cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_solvers test_solvers.cpp)
target_link_libraries(test_solvers cvx_util ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
