#include <cvx/renderer/scene.hpp>
#include "scene_loader.hpp"

#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <boost/filesystem.hpp>

#include <Eigen/Dense>

using namespace std ;
using namespace Eigen ;
using namespace cvx::renderer ;

namespace fs = boost::filesystem ;

static Vector4f color4_to_float4(const aiColor4D &c)
{
    return Vector4f(c.r, c.g, c.b, c.a) ;
}

static void importMaterial(Material &data, const struct aiMaterial *mtl, const boost::filesystem::path &model_path)
{
    data.type_ = Material::PHONG ;

    aiColor4D diffuse, specular, ambient, emission;

    float shininess, strength;
    unsigned int max;

    if ( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
        data.diffuse_ = color4_to_float4(diffuse);

    if( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular) )
        data.specular_ = color4_to_float4(specular);

    if( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient) )
        data.ambient_ = color4_to_float4(ambient);

    if( AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission) )
        data.emission_ = color4_to_float4(emission);

    int shading_model ;
    mtl->Get((const char *)AI_MATKEY_SHADING_MODEL, shading_model);

    switch ( shading_model ) {
    case aiShadingMode_Flat:
        data.type_ = Material::CONSTANT ;
        break ;
    case aiShadingMode_Gouraud:
        data.type_ = Material::LAMBERTIAN ;
        break ;
    case aiShadingMode_Phong:
        data.type_ = Material::PHONG ;
        break ;
    case aiShadingMode_Blinn:
        data.type_ = Material::BLINN ;
        break ;
    default:
        break ;
    }

    max = 1;
    aiReturn ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);

    if ( ret1 == AI_SUCCESS ) {
        max = 1;
        aiReturn ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
        if(ret2 == AI_SUCCESS)
            data.shininess_ =  shininess * strength ;
        else
            data.shininess_ =  shininess ;
    }
    else {
        data.shininess_ = 0 ;
        data.specular_ = Vector4f(0, 0, 0, 0) ;
    }

    int texIndex = 0;
    aiString texPath; //contains filename of texture
    aiTextureMapping tmap ;
    aiTextureMapMode mode ;

    if( AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath, &tmap, 0, 0, 0, &mode) )
    {
        string fileName(texPath.data, texPath.length) ;

        fs::path p = model_path.parent_path() / fileName ;

        if ( fs::exists(p) ) {
            Sampler2D sampler ;
            sampler.image_url_ = p.native() ;
            data.diffuse_ = sampler ;
        }
    }

}


static bool importMaterials(ScenePtr &scene, const string &mpath, const aiScene *sc, map<const aiMaterial *, MaterialPtr> &material_map) {
    for( uint m=0 ; m<sc->mNumMaterials ; m++ ) {
        const aiMaterial *material = sc->mMaterials[m] ;
        MaterialPtr smat(new Material) ;
        importMaterial(*smat, material, mpath) ;
        scene->materials_.push_back(smat) ;
        material_map[material] = smat ;
    }

    return true ;
}

static bool importMeshes(ScenePtr &scene, const aiScene *sc, map<const aiMesh *, MeshPtr> &mesh_map) {

    for( uint m=0 ; m<sc->mNumMeshes ; m++ ) {
        const aiMesh *mesh = sc->mMeshes[m] ;

        if ( mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE ) continue ;

        MeshPtr smesh(new Mesh) ;

        if ( mesh->HasPositions() ) {
            uint n = mesh->mNumVertices ;
            smesh->vertices_.resize(n) ;

            for(int i = 0; i < n; ++i)
                smesh->vertices_[i] = Vector3f(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z) ;
        }

        if ( mesh->HasNormals() ) {
            uint n = mesh->mNumVertices ;
            smesh->normals_.resize(n) ;

            for(int i = 0; i < n; ++i)
                smesh->normals_[i] = Vector3f(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z) ;
        }

        if ( mesh->HasVertexColors(0) ) {
            uint n = mesh->mNumVertices ;
            smesh->colors_.resize(n) ;

            for(int i = 0; i < n; ++i)
                smesh->colors_[i] = Vector3f(mesh->mColors[0][i].r, mesh->mColors[0][i].g, mesh->mColors[0][i].b) ;
        }

        for( uint t=0 ; t<MAX_TEXTURES ; t++ ) {
            uint n = mesh->mNumVertices ;
            if ( mesh->HasTextureCoords(t) ) {
                smesh->tex_coords_[t].resize(n) ;

                for(int i = 0; i < n; ++i)
                    smesh->tex_coords_[t][i] = Vector2f(mesh->mTextureCoords[t][i].x, mesh->mTextureCoords[t][i].y) ;
            }
        }

        if ( mesh->HasFaces() ) {
            uint n = mesh->mNumFaces ;
            smesh->vertex_indices_.resize(n * 3) ;

            for(int i = 0, k=0; i < n ; i++) {
                smesh->vertex_indices_[k++] = mesh->mFaces[i].mIndices[0];
                smesh->vertex_indices_[k++] = mesh->mFaces[i].mIndices[1];
                smesh->vertex_indices_[k++] = mesh->mFaces[i].mIndices[2];
            }
        }

        scene->meshes_.push_back(smesh) ;
        mesh_map[mesh] = smesh ;
    }

    return true ;
}

static bool importLights(ScenePtr &scene, const aiScene *sc) {
    for( uint m=0 ; m<sc->mNumLights ; m++ ) {
        const aiLight *light = sc->mLights[m] ;
        aiNode *lnode = sc->mRootNode->FindNode(light->mName) ;

        LightPtr slight ;

        // find global transformation of the light by accumulating transforms in the scene graph

        Matrix4f tf  = Matrix4f::Identity();

        while ( lnode ) {
            aiMatrix4x4 m = lnode->mTransformation;
            Matrix4f lf ;
            lf << m.a1, m.a2, m.a3, m.a4,
                    m.b1, m.b2, m.b3, m.b4,
                    m.c1, m.c2, m.c3, m.c4,
                    m.d1, m.d2, m.d3, m.d4 ;
            tf *= lf ;
            lnode = lnode->mParent ;
        }

        switch ( light->mType ) {
        case aiLightSource_DIRECTIONAL:
        {
            DirectionalLight *dl = new DirectionalLight ;
            dl->type_ = Light::DIRECTIONAL ;
            Vector4f tdir = tf * Vector4f(light->mDirection.x, light->mDirection.y, light->mDirection.z, 1) ;
            dl->direction_ = tdir.block<3, 1>(0, 0) ;
            dl->color_ << light->mColorDiffuse.r, light->mColorDiffuse.g, light->mColorDiffuse.b ;
            slight.reset(dl) ;
            break ;
        }
        case aiLightSource_POINT:
        {
            PointLight *pl = new PointLight ;
            pl->type_ = Light::POINT ;
            Vector4f tpos = tf * Vector4f(light->mPosition.x, light->mPosition.y, light->mPosition.z, 1) ;
            pl->position_ = tpos.block<3, 1>(0, 0) ;
            pl->color_ << light->mColorDiffuse.r, light->mColorDiffuse.g, light->mColorDiffuse.b ;
            pl->constant_attenuation_ = light->mAttenuationConstant ;
            pl->linear_attenuation_ = light->mAttenuationLinear ;
            pl->quadratic_attenuation_ = light->mAttenuationQuadratic ;

            slight.reset(pl) ;
            break ;

        }
        case aiLightSource_SPOT:
        {
            SpotLight *sl = new SpotLight ;
            sl->type_ = Light::SPOT ;

            Vector4f tpos = tf * Vector4f(light->mPosition.x, light->mPosition.y, light->mPosition.z, 1) ;
            sl->position_ = tpos.block<3, 1>(0, 0) ;

            Vector4f tdir = tf * Vector4f(light->mDirection.x, light->mDirection.y, light->mDirection.z, 1) ;
            sl->direction_ = tdir.block<3, 1>(0, 0) ;

            sl->color_ << light->mColorDiffuse.r, light->mColorDiffuse.g, light->mColorDiffuse.b ;
            sl->constant_attenuation_ = light->mAttenuationConstant ;
            sl->linear_attenuation_ = light->mAttenuationLinear ;
            sl->quadratic_attenuation_ = light->mAttenuationQuadratic ;
            sl->falloff_angle_ = light->mAngleOuterCone ;
            sl->falloff_exponent_ = 0 ;

            slight.reset(sl) ;
            break ;

        }

        }

        if ( slight )
            scene->lights_.push_back(slight) ;
    }



    return true ;
}

static bool importNodes(ScenePtr &scene, NodePtr &pnode, const struct aiScene *sc, const struct aiNode* nd,
                        const map<const aiMesh *, MeshPtr> &meshes,
                        const map<const aiMaterial *, MaterialPtr> &materials)
{
    unsigned int i;
    unsigned int n = 0, t;
    aiMatrix4x4 m = nd->mTransformation;
    /* update transform */

    NodePtr snode(new Node) ;

    Matrix4f tf ;
    tf << m.a1, m.a2, m.a3, m.a4,
            m.b1, m.b2, m.b3, m.b4,
            m.c1, m.c2, m.c3, m.c4,
            m.d1, m.d2, m.d3, m.d4 ;

    snode->mat_ = tf.eval() ;
    snode->name_ = nd->mName.C_Str();

    /* draw all meshes assigned to this node */
    for (; n < nd->mNumMeshes; ++n) {

        GeometryPtr geom(new Geometry) ;

        const aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

        map<const aiMesh *, MeshPtr>::const_iterator mit = meshes.find(mesh) ;

        if ( mit == meshes.end() ) continue ;

        geom->mesh_ = mit->second ;

        const aiMaterial* material = sc->mMaterials[mesh->mMaterialIndex];

        map<const aiMaterial *, MaterialPtr>::const_iterator cit = materials.find(material) ;

        if ( cit != materials.end() )
            geom->material_ = cit->second ;

        snode->geometries_.push_back(geom) ;
    }

    if ( pnode ) {
        pnode->children_.push_back(snode) ;
        snode->parent_ = pnode ;
    }
    else
        scene->nodes_.push_back(snode) ;


    /* draw all children */
    for (n = 0; n < nd->mNumChildren; ++n) {
        if ( !importNodes(scene, snode, sc, nd->mChildren[n], meshes, materials) )
            return false ;
    }

    return true ;
}

static ScenePtr importAssimp(const aiScene *sc, const std::string &fname) {

    map<const aiMesh *, MeshPtr> meshes ;
    map<const aiMaterial *, MaterialPtr> materials ;

    ScenePtr res(new Scene) ;
    if ( !importMeshes(res, sc, meshes) ) return ScenePtr() ;
    if ( !importMaterials(res, fname, sc, materials) ) return ScenePtr() ;
    if ( !importLights(res, sc) ) return ScenePtr() ;

    NodePtr root ;
    if ( !importNodes(res, root, sc, sc->mRootNode, meshes, materials ) ) return ScenePtr() ;

    return res ;
}



namespace cvx { namespace renderer {


ScenePtr Scene::loadAssimp(const std::string &fname) {
    const aiScene *sc = aiImportFile(fname.c_str(), aiProcessPreset_TargetRealtime_MaxQuality | aiProcess_FlipUVs | aiProcess_TransformUVCoords);
    if ( !sc ) {
        throw SceneLoaderException("assimp", aiGetErrorString(), fname) ;
    }

    ScenePtr res = importAssimp(sc, fname) ;

    aiReleaseImport(sc) ;
    return res ;
}

ScenePtr Scene::loadAssimp(const aiScene *sc, const std::string &fname) {
    return importAssimp(sc, fname) ;
}


}
              }
