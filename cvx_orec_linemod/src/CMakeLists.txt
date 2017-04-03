set ( CVX_OBJREC3D_SOURCES
	util.cpp
	train.cpp
	detect.cpp
)

set ( CVX_OBJREC3D_HEADERS
	${CVX_OREC_LINEMOD_INCLUDE_DIR}/linemod.hpp
)
# --z-up --render gouraud --model /home/malasiot/ramcip_ws/src/ramcip_certh/certh_object_recognition/meshes/amita.ply --out /home/malasiot/tmp/render/amita/

add_library(cvx_orec_linemod SHARED ${CVX_OBJREC3D_SOURCES} ${CVX_OBJREC3D_HEADERS})
target_link_libraries(cvx_orec_linemod ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${ASSIMP_LIBRARIES})
