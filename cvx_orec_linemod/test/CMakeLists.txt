add_executable(test_linemod test_linemod.cpp)
target_link_libraries(test_linemod cvx_orec_linemod ${Boost_LIBRARIES} ${catkin_LIBRARIES})


