#ifndef __SCENE_FWD_HPP__
#define __SCENE_FWD_HPP__

#include <boost/shared_ptr.hpp>

namespace cvx { namespace renderer {

struct Material ;
typedef boost::shared_ptr<Material> MaterialPtr ;

struct Mesh ;
typedef boost::shared_ptr<Mesh> MeshPtr ;

struct SkinningModifier ;
typedef boost::shared_ptr<SkinningModifier> SkinningModifierPtr ;

struct Geometry ;
typedef boost::shared_ptr<Geometry> GeometryPtr ;

struct Bone ;
typedef boost::shared_ptr<Bone> BonePtr ;

struct Skeleton ;
typedef boost::shared_ptr<Skeleton> SkeletonPtr ;

struct Node ;
typedef boost::shared_ptr<Node> NodePtr ;

struct Camera ;
typedef boost::shared_ptr<Camera> CameraPtr ;

struct Light ;
typedef boost::shared_ptr<Light> LightPtr ;

class Scene ;
typedef boost::shared_ptr<Scene> ScenePtr ;

}}

#endif
