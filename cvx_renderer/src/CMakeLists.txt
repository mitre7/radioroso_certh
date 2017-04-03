SET(OGL_SHADERS
    shaders/skinning.vs
    shaders/rigid.vs
        shaders/phong.fs
    shaders/flat.fs
        shaders/gouraud.fs
)

add_executable(makeres makeres.cpp)
target_link_libraries(makeres ${Boost_LIBRARIES})

foreach(rsrc ${OGL_SHADERS})
        get_filename_component(RSRC_OUT ${rsrc} NAME_WE)
        add_custom_command(
                OUTPUT ${CMAKE_BINARY_DIR}/${RSRC_OUT}_shader.h
                COMMAND makeres "_rc" ${CMAKE_CURRENT_SOURCE_DIR}/${rsrc} ${CMAKE_BINARY_DIR}/${RSRC_OUT}_shader.h
                DEPENDS ${rsrc})
endforeach()

include_directories(${CMAKE_BINARY_DIR})

set(RENDERER_FILES
	context.cpp
    renderer.cpp
	glsl.cpp
	glsl.hpp
	tools.cpp
	tools.hpp
	scene.cpp
	scene_loader.cpp
	scene_loader.hpp
	obj_loader.hpp
	obj_loader.cpp
	assimp_loader.cpp
	viewpoint_sampler.cpp
    
	${CVX_RENDERER_INCLUDE_DIR}/renderer.hpp
	${CVX_RENDERER_INCLUDE_DIR}/scene.hpp
	${CVX_RENDERER_INCLUDE_DIR}/viewpoint_sampler.hpp

	${CMAKE_BINARY_DIR}/skinning_shader.h
	${CMAKE_BINARY_DIR}/phong_shader.h
	${CMAKE_BINARY_DIR}/flat_shader.h
	${CMAKE_BINARY_DIR}/rigid_shader.h
	${CMAKE_BINARY_DIR}/gouraud_shader.h
)

add_library(cvx_renderer ${RENDERER_FILES})
target_link_libraries(cvx_renderer ${BOOST_LIBARIES} ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW3_LIBRARY} ${GLEW_LIBRARIES} ${catkin_LIBRARIES})
