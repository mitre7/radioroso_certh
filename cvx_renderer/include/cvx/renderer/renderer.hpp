#ifndef __RENDER_SCENE_HPP__
#define __RENDER_SCENE_HPP__

#include <GL/glew.h>
#include <memory>
#include <map>
#include <Eigen/Core>

#include <boost/filesystem.hpp>


#include <cvx/renderer/scene.hpp>

namespace glsl {
    class Program ;
}

namespace cvx { namespace renderer {

class RenderingContext ;
typedef boost::shared_ptr<RenderingContext> RenderingContextPtr ;

class SceneRenderer {
public:

    enum RenderMode { RENDER_FLAT, // no lighting, makes a mask of each object using its diffuse material color
                      RENDER_SMOOTH, // phong
                      RENDER_GOURAUD, // phong
                      RENDER_USER_DEFINED // user defined fragment shader
                    } ;

    SceneRenderer(const ScenePtr &scene, RenderingContextPtr context) ;
    ~SceneRenderer() ;

    bool init() ;
    bool setShader(const char *fragment_shader_code) ;
    void setBackgroundColor(const Eigen::Vector4f &clr) ;


    void render(const Camera &cam, RenderMode mode) ;

    cv::Mat getColor(bool alpha = true);
    cv::Mat getColor(cv::Mat &bg, float alpha);
    cv::Mat getDepth();

private:

    enum VB_TYPES {
        INDEX_BUFFER,
        POS_VB,
        NORMAL_VB,
        COLOR_VB,
        TEXCOORD_VB,
        BONE_VB = TEXCOORD_VB + MAX_TEXTURES,
        TF_VB,
        NUM_VBs
    };

    struct MeshData {
        MeshData() ;
        GLuint buffers_[10];
        GLuint texture_id_, vao_ ;
        GLuint elem_count_ ;
    };

    void release() ;
    void clear(MeshData &data);
    void init_buffers_for_mesh(MeshData &data, Mesh &mesh) ;
    void init_buffers_for_skinning(MeshData &data, SkinningModifier &a) ;
    void render(const NodePtr &node, const Camera &cam, const Eigen::Matrix4f &mat, RenderMode mode) ;
    void render(const GeometryPtr &geom, const Camera &cam, const Eigen::Matrix4f &mat, RenderMode mode) ;
    void set_model_transform(const Eigen::Matrix4f &tf);
    void set_material(const MaterialPtr &material) ;
    void set_program(RenderMode rm) ;
    void set_lights() ;
    void init_textures() ;
    void set_bone_transforms(const SkinningModifierPtr &sk) ;
    bool importAssimpRecursive(const struct aiScene *sc, const boost::filesystem::path &p, const struct aiNode* nd, const Eigen::Matrix4f &ptf) ;

private:

    RenderingContextPtr ctx_ ; // platform specific GL context initialization
    std::map<std::string,boost::shared_ptr<glsl::Program> > programs_ ;
    boost::shared_ptr<glsl::Program> prog_ ;
    std::map<MeshPtr, MeshData> buffers_ ;
    std::map<std::string, GLuint> textures_ ;
    ScenePtr scene_ ;
    Eigen::Matrix4f perspective_, proj_ ;
    GLuint query_ ;
    Eigen::Vector4f bg_clr_ ;
    float znear_, zfar_ ;
    MaterialPtr default_material_ ;
} ;

typedef boost::shared_ptr<SceneRenderer> SceneRendererPtr ;

class RenderingContext {
public:
    RenderingContext(uint32_t width, uint32_t height): width_(width), height_(height) {}
    virtual ~RenderingContext() { }

    virtual void attach() = 0;
    virtual void detach() = 0;

    uint32_t width_, height_ ;
};

// this will return a singleton instance of a rendering context suitable for offscreen rendering

RenderingContextPtr getOffscreenContext(uint32_t width, uint32_t height) ;

}}

#endif
