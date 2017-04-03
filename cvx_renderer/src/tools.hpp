#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include <Eigen/Core>

#include <cvx/renderer/scene.hpp>

// triangulates a 3D (flat) polygon and returns an array of triplets containing indexes to the original polygon
bool triangulate(const std::vector<Eigen::Vector3f> &pts, std::vector<uint32_t> &result) ;

// eliminate indices by duplicating vertices (iterating over the vertices of each triangle)
void flatten_mesh(const cvx::renderer::Mesh &mesh, std::vector<Eigen::Vector3f> &vertices,
                  std::vector<Eigen::Vector3f> &normals, std::vector<Eigen::Vector3f> &colors,
                  std::vector<Eigen::Vector2f> tex_coords[MAX_TEXTURES]) ;


void compute_normals(const std::vector<Eigen::Vector3f> &vertices, const std::vector<uint> &indices, std::vector<Eigen::Vector3f> &vtx_normals) ;


#endif // TOOLS_HPP
