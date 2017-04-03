#include <cvx/renderer/renderer.hpp>

#define GL_GLEXT_PROTOTYPES
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cstring>
#include "shaders.hpp"

#include <Eigen/Dense>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <fstream>

#include "glsl.hpp"
#include "shaders.hpp"
#include "tools.hpp"

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace renderer {


bool SceneRenderer::init() {

    // init stock shaders

    ctx_->attach() ;

    try {
        programs_["rigid_flat"] = boost::make_shared<glsl::Program>(rigid_vertex_shader_rc, flat_fragment_shader_rc) ;
        programs_["rigid_smooth"] = boost::make_shared<glsl::Program>(rigid_vertex_shader_rc, phong_fragment_shader_rc) ;
        programs_["rigid_gouraud"] = boost::make_shared<glsl::Program>(rigid_vertex_shader_rc, gouraud_fragment_shader_rc) ;

        programs_["skinning_flat"] = boost::make_shared<glsl::Program>(skinning_vertex_shader_rc, flat_fragment_shader_rc) ;
        programs_["skinning_smooth"] = boost::make_shared<glsl::Program>(skinning_vertex_shader_rc, phong_fragment_shader_rc) ;
        programs_["skinning_gouraud"] = boost::make_shared<glsl::Program>(skinning_vertex_shader_rc, gouraud_fragment_shader_rc) ;
    }
    catch ( const glsl::Error &e ) {
        cerr << e.what() << endl ;
        return false ;
    }

    // create vertex buffers

    for( uint m=0 ; m<scene_->meshes_.size() ; m++ )
    {
        MeshData data ;
        MeshPtr mesh = scene_->meshes_[m] ;
        init_buffers_for_mesh(data, *scene_->meshes_[m]);
        buffers_[mesh] = data ;
    }

    for( uint m=0 ; m<scene_->armatures_.size() ; m++ )
    {
        SkinningModifierPtr arm = scene_->armatures_[m] ;
        std::map<MeshPtr, MeshData>::iterator it = buffers_.find(arm->skin_) ;
        if ( it != buffers_.end() ) {
            MeshData &data = it->second ;
            init_buffers_for_skinning(data, *arm);
        }
    }

    // load textures

    init_textures() ;

    default_material_.reset(new Material) ;

    default_material_->type_ = Material::PHONG ;
    default_material_->ambient_ = Vector4f(0.0, 0.0, 0.0, 1) ;
    default_material_->diffuse_ = Vector4f(0.5, 0.5, 0.5, 1) ;

    return true ;
}

bool SceneRenderer::setShader(const char *fragment_shader_code)
{
    try {
        programs_["rigid_user"] = boost::make_shared<glsl::Program>(rigid_vertex_shader_rc, fragment_shader_code) ;
        programs_["skinning_user"] = boost::make_shared<glsl::Program>(skinning_vertex_shader_rc, fragment_shader_code) ;
        return true ;
    }
    catch ( const glsl::Error &e) {
        cerr << e.what() << endl ;
        return false ;
    }
}

void SceneRenderer::setBackgroundColor(const Vector4f &clr)
{
    bg_clr_ = clr ;
}

void SceneRenderer::release() {

}


void SceneRenderer::clear(MeshData &data) {
    glDeleteVertexArrays(1, &data.vao_) ;
}

#define POSITION_LOCATION    0
#define NORMALS_LOCATION    1
#define COLORS_LOCATION    2
#define BONE_ID_LOCATION    3
#define BONE_WEIGHT_LOCATION    4
#define UV_LOCATION 5

SceneRenderer::MeshData::MeshData() {}

void SceneRenderer::init_buffers_for_mesh(MeshData &data, Mesh &mesh)
{
    // Create the VAO
    glGenVertexArrays(1, &data.vao_);
    glBindVertexArray(data.vao_);

    vector<Vector3f> vertices, normals, colors ;
    vector<Vector2f> tex_coords[MAX_TEXTURES] ;
    vector<GLuint> indices ;

    flatten_mesh(mesh, vertices, normals, colors, tex_coords) ;
    data.elem_count_ = vertices.size() ;

    glGenBuffers(1, &data.buffers_[POS_VB]);
    glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[POS_VB]);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat) * 3, &vertices[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(POSITION_LOCATION);
    glVertexAttribPointer(POSITION_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    if ( !normals.empty() ) {
        glGenBuffers(1, &data.buffers_[NORMAL_VB]);
        glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[NORMAL_VB]);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLfloat) * 3, (GLfloat *)normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(NORMALS_LOCATION);
        glVertexAttribPointer(NORMALS_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    }

    if ( !colors.empty() ) {
        glGenBuffers(1, &data.buffers_[COLOR_VB]);
        glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[COLOR_VB]);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat) * 3, (GLfloat *)colors.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(COLORS_LOCATION);
        glVertexAttribPointer(COLORS_LOCATION, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    }

    for( uint t = 0 ; t<MAX_TEXTURES ; t++ ) {
        if ( !tex_coords[t].empty() ) {
            glGenBuffers(1, &data.buffers_[TEXCOORD_VB + t]);
            glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[TEXCOORD_VB + t]);
            glBufferData(GL_ARRAY_BUFFER, tex_coords[t].size() * sizeof(GLfloat) * 2, (GLfloat *)tex_coords[t].data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(UV_LOCATION + t);
            glVertexAttribPointer(UV_LOCATION + t, 2, GL_FLOAT, GL_FALSE, 0, NULL);
        }
    }

    if ( !indices.empty() ) {
        glGenBuffers(1, &data.buffers_[INDEX_BUFFER]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data.buffers_[INDEX_BUFFER]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
    }

    glGenBuffers(1, &data.buffers_[TF_VB]);
    glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[TF_VB]);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat) * 3, 0, GL_STATIC_READ);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void SceneRenderer::init_buffers_for_skinning(SceneRenderer::MeshData &data, SkinningModifier &armature)
{
    std::vector<SkinningModifier::VertexBoneData> &bone_weights = armature.vertex_weights_ ;
    std::vector<SkinningModifier::VertexBoneData> flat_weights ;

    for( uint v=0 ; v<armature.skin_->vertex_indices_.size() ; v++) {
        uint32_t vidx = armature.skin_->vertex_indices_[v] ;
        const SkinningModifier::VertexBoneData &vb_data = bone_weights[vidx] ;
        flat_weights.push_back(vb_data) ;
    } ;

    glBindVertexArray(data.vao_);

    glGenBuffers(1, &data.buffers_[BONE_VB]);
    glBindBuffer(GL_ARRAY_BUFFER, data.buffers_[BONE_VB]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(flat_weights[0]) * flat_weights.size(), &flat_weights[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(BONE_ID_LOCATION);
    glVertexAttribIPointer(BONE_ID_LOCATION, 4, GL_INT, sizeof(SkinningModifier::VertexBoneData), (const GLvoid*)0);

    glEnableVertexAttribArray(BONE_WEIGHT_LOCATION);
    glVertexAttribPointer(BONE_WEIGHT_LOCATION, 4, GL_FLOAT, GL_FALSE, sizeof(SkinningModifier::VertexBoneData), (const GLvoid*)16);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

}

static Matrix4f perspective(float fovy, float aspect, float zNear, float zFar)	{
    assert(abs(aspect - std::numeric_limits<float>::epsilon()) > static_cast<float>(0));

    float const d = 1/tan(fovy / static_cast<float>(2));

    Matrix4f Result ;
    Result.setZero() ;

    Result(0, 0) = d / aspect ;
    Result(1, 1) = d ;
    Result(2, 2) =  (zFar + zNear) / (zNear - zFar);
    Result(2, 3) =  2 * zFar * zNear /(zNear - zFar) ;
    Result(3, 2) = -1 ;

    return Result;
}

void SceneRenderer::render(const Camera &cam, RenderMode mode) {

    glEnable(GL_DEPTH_TEST) ;

    glEnable(GL_CULL_FACE) ;
    glCullFace(GL_BACK) ;
    glFrontFace(GL_CCW) ;
    glClearColor(bg_clr_.x(), bg_clr_.y(), bg_clr_.z(), bg_clr_.w()) ;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    if ( cam.type_ == Camera::PERSPECTIVE ) {
        const PerspectiveCamera &pcam = (const PerspectiveCamera &)cam ;
        perspective_ = perspective(pcam.yfov_.get(), pcam.aspect_ratio_.get(), pcam.znear_.get(), pcam.zfar_.get()) ;
        znear_ = pcam.znear_.get() ;
        zfar_ = pcam.zfar_.get() ;
    }

    glViewport(0, 0, ctx_->width_, ctx_->height_);

    proj_ = cam.mat_ ;

    // render node hierarchy

    for( uint i=0 ; i<scene_->nodes_.size() ; i++ ) {
        const NodePtr &n = scene_->nodes_[i] ;
        render(n, cam, Matrix4f::Identity(), mode) ;
    }
}

void SceneRenderer::render(const NodePtr &node, const Camera &cam, const Matrix4f &tf, RenderMode mode) {
    Matrix4f mat = node->mat_,
    tr = tf * mat ; // accumulate transform

    for( uint i=0 ; i<node->geometries_.size() ; i++ ) {
        const GeometryPtr &m = node->geometries_[i] ;
        render(m, cam, tr, mode) ;
    }

    for( uint i=0 ; i<node->children_.size() ; i++ ) {
        const NodePtr &n = node->children_[i] ;
        render(n, cam, tr, mode) ;
    }
}


SceneRenderer::SceneRenderer(const ScenePtr &scene, RenderingContextPtr ctx): scene_(scene), ctx_(ctx)
{
    bg_clr_ << 0, 0, 0, 1 ;
    init() ;
}

SceneRenderer::~SceneRenderer() {
    release() ;
}

void SceneRenderer::set_model_transform(const Matrix4f &tf)
{
    Matrix4f mvp =  perspective_ * proj_ * tf;
    Matrix4f mv =   proj_ * tf;
    Matrix3f wp = mv.block<3, 3>(0, 0).transpose().inverse() ;

    prog_->setUniform("gProj", mvp) ;
    prog_->setUniform("gModel", mv) ;
    prog_->setUniform("gNormal", wp) ;
}

void SceneRenderer::set_material(const MaterialPtr &material)
{

    if (  material->ambient_ && material->ambient_.get().which() == 0 ) {
        Vector4f clr = boost::get<Vector4f>(material->ambient_.get()) ;
        prog_->setUniform("g_material.ambient", clr) ;
    }
    else
        prog_->setUniform("g_material.ambient", Vector4f(0, 0, 0, 1)) ;

    if (  material->diffuse_ ) {
        if ( material->diffuse_.get().which() == 0 ) {
            Vector4f clr = boost::get<Vector4f>(material->diffuse_.get()) ;
            prog_->setUniform("g_material.diffuse", clr) ;
            prog_->setUniform("g_material.diffuse_map", false) ;
        }
        else {
            Sampler2D &ts = boost::get<Sampler2D>(material->diffuse_.get()) ;

            prog_->setUniform("texUnit", 0) ;
            prog_->setUniform("g_material.diffuse_map", true) ;

            if ( textures_.count(ts.image_url_) ) {
                glBindTexture(GL_TEXTURE_2D, textures_[ts.image_url_]) ;
            }
        }
    }
    else
        prog_->setUniform("g_material.diffuse", Vector4f(0.5, 0.5, 0.5, 1)) ;


    if (  material->specular_ && material->specular_.get().which() == 0 ) {
        Vector4f clr = boost::get<Vector4f>(material->specular_.get()) ;
        prog_->setUniform("g_material.specular", clr) ;
    }
    else
        prog_->setUniform("g_material.specular", Vector4f(0.5, 0.5, 0.5, 1)) ;

    if (  material->shininess_ )
        prog_->setUniform("g_material.shininess", material->shininess_.get()) ;
    else
        prog_->setUniform("g_material.shininess", 0) ;
}

#define MAX_LIGHTS 10

void SceneRenderer::set_lights()
{
    for( uint i=0 ; i< MAX_LIGHTS ; i++  ) {

        string vname = str( boost::format("g_light_source[%d]") % i ) ;

        if ( i >= scene_->lights_.size() ) {
            prog_->setUniform(vname + ".light_type", -1) ;
            continue ;
        }

        const LightPtr &light = scene_->lights_[i] ;

        if ( light->type_ == Light::AMBIENT ) {
            const AmbientLight &alight = (const AmbientLight &)*light ;

            prog_->setUniform(vname + ".light_type", 0) ;
            prog_->setUniform(vname + ".color", alight.color_) ;
        }
        else if ( light->type_ == Light::DIRECTIONAL ) {
            const DirectionalLight &dlight = (const DirectionalLight &)*light ;

            prog_->setUniform(vname + ".light_type", 1) ;
            prog_->setUniform(vname + ".color", dlight.color_) ;
            prog_->setUniform(vname + ".direction", dlight.direction_) ;
        }
        else if ( light->type_ == Light::SPOT) {
            const SpotLight &slight = (const SpotLight &)*light ;

            prog_->setUniform(vname + ".light_type", 2) ;
            prog_->setUniform(vname + ".color", slight.color_) ;
            prog_->setUniform(vname + ".direction", slight.direction_) ;
            prog_->setUniform(vname + ".position", slight.position_) ;
            prog_->setUniform(vname + ".constant_attenuation", slight.constant_attenuation_) ;
            prog_->setUniform(vname + ".linear_attenuation", slight.linear_attenuation_) ;
            prog_->setUniform(vname + ".quadratic_attenuation", slight.quadratic_attenuation_) ;
            prog_->setUniform(vname + ".spot_exponent", slight.falloff_exponent_) ;
            prog_->setUniform(vname + ".spot_cos_cutoff", (float)cos(M_PI*slight.falloff_angle_/180.0)) ;
        }
        else if ( light->type_ == Light::POINT) {
            const PointLight &plight = (const PointLight &)*light ;

            prog_->setUniform(vname + ".light_type", 3) ;
            prog_->setUniform(vname + ".color", plight.color_) ;
            prog_->setUniform(vname + ".position", plight.position_) ;
            prog_->setUniform(vname + ".constant_attenuation", plight.constant_attenuation_) ;
            prog_->setUniform(vname + ".linear_attenuation", plight.linear_attenuation_) ;
            prog_->setUniform(vname + ".quadratic_attenuation", plight.quadratic_attenuation_) ;
        }
    }

}

void SceneRenderer::set_bone_transforms(const SkinningModifierPtr &skm) {

    // global bone map (e.g. multiple skeletons)

    map<string, BonePtr> bone_idx ;
    for( uint i=0 ; i<skm->skeletons_.size() ; i++ ) {
        SkeletonPtr sk = skm->skeletons_[i] ;
        for( uint j=0 ; j<sk->bones_.size() ; j++ ) {
            BonePtr b = sk->bones_[j] ;
            bone_idx[b->name_] = b ;
        }
    }

    for( uint i=0 ; i<skm->joint_names_.size() ; i++ ) {

        string var_name = str ( boost::format("gBones[%d]") % i ) ;

        BonePtr b = bone_idx[skm->joint_names_[i]] ;

        // accumulate transforms to skeleton root

        Matrix4f acc = Matrix4f::Identity() ;
        do {
            acc =  b->mat_ * acc ;
            b = b->parent_ ;
        } while ( b) ;

        Matrix4f m =   acc  * skm->inv_bind_matrices_[i] ;

        prog_->setUniform(var_name, m) ;
    }

}

void SceneRenderer::init_textures()
{
    for( uint i=0 ; i<scene_->materials_.size() ; i++ ) {

        MaterialPtr &m = scene_->materials_[i] ;

        if ( m->diffuse_ && m->diffuse_.get().which() == 1 ) {
            Sampler2D &texture = boost::get<Sampler2D>(m->diffuse_.get()) ;

            cv::Mat image = cv::imread(texture.image_url_) ;

            if ( image.data ) {

//                cv::flip(image, image, 0) ;

                GLuint texture_id ;

                glActiveTexture(GL_TEXTURE0);
                glGenTextures(1, &texture_id);
                glBindTexture(GL_TEXTURE_2D, texture_id);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

                // Set texture clamping method
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

                glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
                glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
                glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
                glPixelStorei( GL_UNPACK_SKIP_ROWS, 0 );


                glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                             0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                             GL_RGB,            // Internal colour format to convert to
                             image.cols,
                             image.rows,
                             0,                 // Border width in pixels (can either be 1 or 0)
                             GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                             GL_UNSIGNED_BYTE,  // Image data type
                             image.ptr());        // The actual image data itself

                glGenerateMipmap(GL_TEXTURE_2D);

                textures_[texture.image_url_] = texture_id ;
            }
        }
    }
}


void SceneRenderer::render(const GeometryPtr &geom, const Camera &cam, const Matrix4f &mat, RenderMode mode)
{
    MeshData &data = buffers_[geom->mesh_] ;

    bool do_skinning = (bool)geom->skin_modifier_  ;

    if ( !do_skinning ) {
        if ( mode == RENDER_FLAT )
            prog_ = programs_["rigid_flat"] ;
        else if ( mode == RENDER_SMOOTH )
            prog_ = programs_["rigid_smooth"] ;
        else if ( mode == RENDER_GOURAUD )
            prog_ = programs_["rigid_gouraud"] ;
        else if ( mode == RENDER_USER_DEFINED )
            prog_ = programs_["rigid_user"] ;
    }
    else {
        if ( mode == RENDER_FLAT )
            prog_ = programs_["skinning_flat"] ;
        else if ( mode == RENDER_SMOOTH )
            prog_ = programs_["skinning_smooth"] ;
        else if ( mode == RENDER_GOURAUD )
            prog_ = programs_["skinning_gouraud"] ;
        else if ( mode == RENDER_USER_DEFINED )
            prog_ = programs_["skinning_user"] ;
    }

    assert( prog_ ) ;

    prog_->use() ;

    set_model_transform(mat) ;
    if ( do_skinning ) set_bone_transforms(geom->skin_modifier_);
    if ( geom->material_) set_material(geom->material_) ;
    else set_material(default_material_) ;
    set_lights() ;

#if 0
    glBindVertexArray(data.vao_);
    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, data.buffers_[TF_VB]);

    glEnable(GL_RASTERIZER_DISCARD);



    glBeginTransformFeedback(GL_TRIANGLES);


    glBeginQuery( GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query_ );
    glDrawArrays(GL_TRIANGLES, 0, data.elem_count_) ;
    glEndQuery( GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN );
    glEndTransformFeedback();

    GLuint primitives_written = 0 ;
    glGetQueryObjectuiv( query_, GL_QUERY_RESULT, &primitives_written );



    glDisable(GL_RASTERIZER_DISCARD);

    glFlush();

    vector<GLfloat> fdata(36, 0) ;

    glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 36*sizeof(GLfloat), fdata.data());

    glBindVertexArray(0) ;
#else
    glBindVertexArray(data.vao_);
    glDrawArrays(GL_TRIANGLES, 0, data.elem_count_) ;
    glBindVertexArray(0) ;

    glFlush();
#endif
    glUseProgram(0) ;
}

cv::Mat SceneRenderer::getColor(bool alpha)
{
    if ( alpha )
    {
        cv::Mat_<cv::Vec4b> mask(ctx_->height_, ctx_->width_) ;
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, ctx_->width_, ctx_->height_, GL_RGBA, GL_UNSIGNED_BYTE, mask.ptr());

        cv::flip(mask, mask, 0) ;

        cv::cvtColor(mask, mask, CV_RGBA2BGRA);

        return mask ;
    }
    else
    {
        cv::Mat_<cv::Vec3b> mask(ctx_->height_, ctx_->width_) ;
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, ctx_->width_, ctx_->height_, GL_RGB, GL_UNSIGNED_BYTE, mask.ptr());

        cv::flip(mask, mask, 0) ;

        cv::cvtColor(mask, mask, CV_RGB2BGR);

        return mask ;
    }
}

cv::Mat SceneRenderer::getColor(cv::Mat &bg, float alpha)
{
    cv::Mat_<cv::Vec3b> mask(ctx_->height_, ctx_->width_) ;
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, ctx_->width_, ctx_->height_, GL_RGB, GL_UNSIGNED_BYTE, mask.ptr());
    cv::flip(mask, mask, 0) ;
    cv::cvtColor(mask, mask, CV_RGB2BGR);

    cv::Mat dst ;
    cv::addWeighted( mask, alpha, bg, (1 - alpha), 0.0, dst);

    return dst ;
}

cv::Mat SceneRenderer::getDepth() {

    cv::Mat_<float> depth(ctx_->height_, ctx_->width_);

    glReadBuffer(GL_DEPTH_ATTACHMENT);

    glReadPixels(0, 0, ctx_->width_, ctx_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

    cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
    float max_allowed_z = zfar_ * 0.99;

    unsigned int i_min = ctx_->width_, i_max = 0, j_min = ctx_->height_, j_max = 0;

    for (unsigned int j = 0; j < ctx_->height_; ++j)
        for (unsigned int i = 0; i < ctx_->width_; ++i, ++it)
        {
            //need to undo the depth buffer mapping
            //http://olivers.posterous.com/linear-depth-in-glsl-for-real
            float z  = 2 * zfar_ * znear_ / (zfar_ + znear_ - (zfar_ - znear_) * (2 * (*it) - 1));

            if (z > max_allowed_z) *it = 0;
            else {
                *it = z ;
            }
        }
    // Rescale the depth to be in millimeters
    cv::Mat depth_scale(cv::Size(ctx_->width_, ctx_->height_), CV_16UC1);
    depth.convertTo(depth_scale, CV_16UC1, 1e3);

    cv::flip(depth_scale, depth_scale, 0) ;

    // cv::imwrite("/tmp/dmap.png", vl::depthViz(depth_scale)) ;
    return depth_scale;

}


}}
