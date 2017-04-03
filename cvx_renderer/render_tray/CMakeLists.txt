add_executable(render_tray render_tray.cpp)
target_link_libraries(render_tray cvx_renderer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
