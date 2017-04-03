add_executable(test_render test_scene_render.cpp)
target_link_libraries(test_render cvx_renderer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
