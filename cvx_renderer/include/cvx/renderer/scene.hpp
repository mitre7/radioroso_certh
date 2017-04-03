#ifndef __SCENE_HPP__
#define __SCENE_HPP__

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>

#include <cvx/util/camera/camera.hpp>
#include <cvx/renderer/scene_fwd.hpp>

#include <assimp/scene.h>

namespace cvx { namespace renderer {

// class defining a complete scene

class Scene {
public:

    // load scene from file
    static ScenePtr load(const std::string &fname) ;
    static ScenePtr loadAssimp(const std::string &fname) ;
    static ScenePtr loadAssimp(const aiScene *sc, const std::string &fname) ;

    // add light to scene
    void addLight(const LightPtr &light) {
        lights_.push_back(light) ;
    }

public:

    std::vector<NodePtr> nodes_ ;           // root nodes of the hierarchy
    std::vector<MaterialPtr> materials_ ;   // shared materials
    std::vector<MeshPtr> meshes_ ;          // shared meshes
    std::vector<SkinningModifierPtr> armatures_ ; // skinning data
    std::vector<CameraPtr> cameras_ ;       // list of cameras
    std::vector<LightPtr> lights_ ;         // list of lights
};

// a hieracrchy of nodes. each node applies a transformation to the attached geometries

struct Node {
public:

    Node(): mat_(Eigen::Matrix4f::Identity()) {}
    std::string name_ ;
    Eigen::Matrix4f mat_ ;                 // transformation matrix to apply to child nodes and attached geometries
    std::vector<NodePtr> children_ ;       // child nodes
    std::vector<GeometryPtr> geometries_ ; // meshes associated with this node

    NodePtr parent_ ;                      // parent node
};


#define MAX_TEXTURES 4

// mesh data of triangular mesh

struct Mesh {
    // vertex attribute arrays, they can be of different size if attributes (e.g. normals) are shared for multiple vertices (e.g. per face normals)
    std::vector<Eigen::Vector3f> vertices_, normals_, colors_ ;
    std::vector<Eigen::Vector2f> tex_coords_[MAX_TEXTURES] ;

    // triplets of vertex attribute indices corresponding to the triangles of the mesh (all same size)
    std::vector<uint32_t> vertex_indices_, normal_indices_, color_indices_, tex_coord_indices_[MAX_TEXTURES] ;
};


// texture and its parameters
struct Sampler2D {
    std::string image_url_ ;
    std::string wrap_s_, wrap_t_ ;
};

typedef boost::variant<Eigen::Vector4f, Sampler2D> ColorOrTexture ;

struct Material {
    enum Type { PHONG, LAMBERTIAN, BLINN, CONSTANT } ;

    std::string name_ ;
    Type type_ ;        // type of material
    boost::optional<ColorOrTexture> emission_, ambient_, diffuse_,
    specular_, reflective_, transparent_ ; // material component color or associated texture map

    boost::optional<float> reflectivity_, shininess_, transparency_ ;
};


// Geometry structure associates a mesh instance with material and armature.

struct Geometry {
    MeshPtr mesh_ ;
    MaterialPtr material_ ;
    SkinningModifierPtr skin_modifier_ ;
};

// A bone is defined by a transform from the root space to the bone space

struct Bone {
    Bone(): mat_(Eigen::Matrix4f::Identity()) {}

    std::string name_ ;
    Eigen::Matrix4f mat_ ; // relative transform between current node and its parent

    BonePtr parent_ ;
    std::vector<BonePtr> children_ ;
};

// a hiearchy of bones

struct Skeleton {
    std::map<std::string, uint> bone_idx_ ; // helper map to find bone by name
    std::vector<BonePtr> bones_ ;           // list of bones
    BonePtr root_ ;                         // skeleton root

    int getBoneIndex(const std::string &name) {
        std::map<std::string, uint>::const_iterator it = bone_idx_.find(name) ;
        if ( it == bone_idx_.end() ) return -1 ;
        else return it->second ;
    }
};

// a skinning modifier consists of a list of skeletons, participating bones and associated bind matrices and vertex weights

struct SkinningModifier {
public:

#define MAX_BONES_PER_VERTEX 4

    struct VertexBoneData
    {
        int id_[MAX_BONES_PER_VERTEX];
        float weight_[MAX_BONES_PER_VERTEX];
    };

    MeshPtr skin_ ;                                   // The base mesh
    std::vector<SkeletonPtr> skeletons_ ;             // List of skeletons
    std::vector<VertexBoneData> vertex_weights_ ;     // Vertex weights, list of (bone index, weight) pairs for each vertex of the mesh.
                                                      // Indices point to joint_names_ array.
    std::vector<std::string> joint_names_ ;           // List of joints that participate in the skinning transformation
    std::vector<Eigen::Matrix4f> inv_bind_matrices_ ; // The associated inverse bind matrices for eacj joint
};

// Abstract camera

struct Camera {
public:
    enum Type { PERSPECTIVE, ORTHOGRAPHIC } ;

    Camera(Type t, const Eigen::Matrix4f &m = Eigen::Matrix4f::Identity()): type_(t), mat_(m) {}

    Type type_ ;
    Eigen::Matrix4f mat_ ; // view transformation
};

// Perspective camera

struct PerspectiveCamera: public Camera {

    PerspectiveCamera(): Camera(PERSPECTIVE) {}
    PerspectiveCamera(const cvx::util::PinholeCamera &cam, const Eigen::Matrix4f &pos = Eigen::Matrix4f::Identity()): Camera(PERSPECTIVE, pos) {
        aspect_ratio_ = cam.width() / (float)cam.height() ;
        yfov_ = 2 * atan( cam.height() / cam.fy()/2.0)  ;
        xfov_ = aspect_ratio_.get() * yfov_.get() ;
        znear_ = 0.01 ;
        zfar_ = 10 ;
    }


    boost::optional<float> xfov_, yfov_, aspect_ratio_, znear_, zfar_ ;
};

// Orthographic camera

struct OrthographicCamera: public Camera {
    OrthographicCamera(): Camera(ORTHOGRAPHIC) {}

    boost::optional<float> xmag_, ymag_, znear_, zfar_ ;
};

// Abstract light

struct Light {
    enum Type { AMBIENT, POINT, DIRECTIONAL, SPOT } ;
    Light(Type t): type_(t) {}

    Type type_ ;
};

struct AmbientLight: public Light {
    AmbientLight(): Light(AMBIENT) {}
    AmbientLight(const Eigen::Vector3f &clr): Light(AMBIENT), color_(clr) {}

    Eigen::Vector3f color_ ;
};

struct PointLight: public Light {
    PointLight(): Light(POINT), constant_attenuation_(1.0), linear_attenuation_(0.0) {}
    PointLight(const Eigen::Vector3f &pos, const Eigen::Vector3f &clr): Light(POINT), position_(pos), color_(clr), constant_attenuation_(1.0), linear_attenuation_(0.0) {}

    Eigen::Vector3f position_ ;
    Eigen::Vector3f color_ ;
    float constant_attenuation_ ;
    float linear_attenuation_ ;
    float quadratic_attenuation_ ;
};

struct DirectionalLight: public Light {
    DirectionalLight(): Light(DIRECTIONAL) {}
    DirectionalLight(const Eigen::Vector3f &dir, const Eigen::Vector3f &clr): Light(DIRECTIONAL), direction_(dir), color_(clr) {}

    Eigen::Vector3f color_, direction_ ;
};

struct SpotLight: public Light {
    SpotLight(): Light(SPOT), constant_attenuation_(1.0),
    linear_attenuation_(0.0), quadratic_attenuation_(0.0),
    falloff_angle_(M_PI), falloff_exponent_(0){}

    Eigen::Vector3f color_, direction_, position_ ;
    float constant_attenuation_ ;
    float linear_attenuation_ ;
    float quadratic_attenuation_ ;
    float falloff_angle_, falloff_exponent_ ;
};


}}
#endif
