#include "obj_loader.hpp"
#include "tools.hpp"

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#include <Eigen/Core>

#include <fstream>

using namespace std ;
using namespace Eigen ;
using namespace cvx::renderer ;

OBJSceneLoader::OBJSceneLoader() {}

bool OBJSceneLoader::canLoad(const string &fname) const {
    try {
      load_obj(fname, 10) ;
      return true ;
    }
    catch ( SceneLoaderException & ) {
        return false ;
    }
}

ScenePtr OBJSceneLoader::load(const string &fname) {
    return load_obj(fname) ;
}

static bool parse_float(const std::string &t, float &val) {
    try {
        val = boost::lexical_cast<float>(t) ;
        return true ;
    }
    catch ( boost::bad_lexical_cast & ) {
        return false ;
    }
}

static bool parse_int(const std::string &t, int &val) {
    try {
        val = boost::lexical_cast<int>(t) ;
        return true ;
    }
    catch ( boost::bad_lexical_cast & ) {
        return false ;
    }
}

ScenePtr OBJSceneLoader::load_obj_stream(istream &strm, const std::string &fname, int max_lines) const {
    if ( !strm ) {
        throw SceneLoaderException("OBJDriver", "cannot open file for reading", fname) ;
    }

    ScenePtr scene(new Scene) ;

    typedef boost::tokenizer<boost::char_separator<char> >  stokenizer;

    uint line_no = 0 ;

    vector<MeshPtr> meshes ;
    vector<string> onames ;

    MeshPtr mesh ;
    string oname = "mesh" ;

    while ( strm )
    {
        string line ;
        getline(strm, line) ;
        line_no++ ;

        if ( line[0]=='#' || line[0]=='$' || line.empty() ) continue ;

        vector<string> tokens ;
        typedef boost::tokenizer<boost::char_separator<char> >  stokenizer;

        boost::char_separator<char> sep("\t\r ");
        stokenizer tok(line, sep);

        for (stokenizer::iterator tok_iter = tok.begin(); tok_iter != tok.end(); ++tok_iter )
             if ( !tok_iter->empty() ) tokens.push_back(*tok_iter) ;

        if ( tokens.empty() ) continue ;

        if ( tokens[0] == "v" ) // vertex
        {
            float x, y, z, w = 1.0, r, g, b ;

            if ( !mesh ) {
                mesh.reset(new Mesh) ;
                meshes.push_back(mesh) ;
                onames.push_back(oname) ;
            }

            if ( tokens.size() < 4 )
                throw SceneLoaderException("OBJDriver", "error parsing OBJ file", fname, line_no) ;

            if ( tokens.size() == 4 && parse_float(tokens[1], x) && parse_float(tokens[2], y) && parse_float(tokens[3], z) )
                mesh->vertices_.push_back(Vector3f(x, y, z)) ;
            else  if ( tokens.size() == 5 && parse_float(tokens[1], x) && parse_float(tokens[2], y) && parse_float(tokens[3], z) && parse_float(tokens[4], w) )
                mesh->vertices_.push_back(Vector3f(x/w, y/w, z/w)) ;
            else if ( tokens.size() == 7 && parse_float(tokens[1], x) && parse_float(tokens[2], y) && parse_float(tokens[3], z)
                                      && parse_float(tokens[4], r) && parse_float(tokens[5], g) && parse_float(tokens[6], b) )  {
                mesh->vertices_.push_back(Vector3f(x, y, z)) ;
                mesh->colors_.push_back(Vector3f(r, g, b)) ;
            }
            else
                throw SceneLoaderException("OBJDriver", "error parsing OBJ file", fname, line_no) ;

        }
        else if ( tokens[0] == "vn" ) // normal
        {
            float x, y, z ;

            if ( tokens.size() < 3 ||
                 ( !parse_float(tokens[1], x) || !parse_float(tokens[2], y) || !parse_float(tokens[3], z) ) ) {
                throw SceneLoaderException("OBJDriver", "error parsing OBJ file", fname, line_no) ;
            }

            if ( !mesh ) {
                mesh.reset(new Mesh) ;
                meshes.push_back(mesh) ;
                onames.push_back(oname) ;
            }
            mesh->normals_.push_back(Vector3f(x, y, z)) ;
        }
        else if ( tokens[0] == "vt" ) // normal
        {
            float u, v ;

            if ( tokens.size() < 2 ||
                 ( !parse_float(tokens[1], u) || !parse_float(tokens[2], v) ) ) {
                throw SceneLoaderException("OBJDriver", "error parsing OBJ file", fname, line_no) ;
            }

            if ( !mesh ) {
                mesh.reset(new Mesh) ;
                meshes.push_back(mesh) ;
                onames.push_back(oname) ;
            }

            mesh->tex_coords_[0].push_back(Vector2f(u, v)) ;
        }
        else if ( tokens[0] == "f" )
        {
            if ( !mesh ) {
                mesh.reset(new Mesh) ;
                meshes.push_back(mesh) ;
                onames.push_back(oname) ;
            }

            std::vector<int> vidxs, tidxs, nidxs ;
            std::vector<Vector3f> vertices ;
            bool error = false ;

            for ( uint i=1 ; i<tokens.size() ; i++ ) {

                vector<string> index_tokens ;
                boost::algorithm::split(index_tokens, tokens[i], boost::is_any_of("/"));

                int vi = -1, ni = -1, ti = -1 ;

                if ( index_tokens.size() == 0 ) { error = true ; break ; }
                if ( index_tokens.size() > 0 && !parse_int(index_tokens[0], vi)) { error = true ; break ; }
                if ( index_tokens.size() > 1 && !index_tokens[1].empty() && !parse_int(index_tokens[1], ti) ) {
                    error = true ; break ;
                }
                if ( index_tokens.size() > 2 && !parse_int(index_tokens[2], ni) ) {
                    error = true ; break ;
                }

                if ( vi != -1 ) {
                    vidxs.push_back(vi-1) ;
                    vertices.push_back(mesh->vertices_[vi-1]) ;
                }
                if ( ti != -1 ) tidxs.push_back(ti-1) ;
                if ( ni != -1 ) nidxs.push_back(ni-1) ;
            }

            vector<uint32_t> triangles ;
            triangulate(vertices, triangles) ;

            for(uint k=0 ; k<triangles.size() ; k++ ) {
                uint32_t vidx = triangles[k] ;
                if ( !vidxs.empty() )
                    mesh->vertex_indices_.push_back(vidxs[vidx]) ;
                if ( !nidxs.empty() )
                    mesh->normal_indices_.push_back(nidxs[vidx]) ;
                if ( !tidxs.empty() )
                     mesh->tex_coord_indices_[0].push_back(tidxs[vidx]) ;
            }
        }
        else if ( tokens[0] == "usemtl" ) continue ;
        else if ( tokens[0] == "mtllib" ) continue ;
        else if ( tokens[0] == "o" ) {
            oname = tokens[1] ; // update current node
        }
        else if ( tokens[0] == "g" ) continue ;
        else if ( tokens[0] == "s" ) continue ;
        else {
           throw SceneLoaderException("OBJDriver", "error parsing OBJ file", fname, line_no) ;
        }

        if ( max_lines > 0 && line_no > max_lines ) return scene ;
    }

    for( uint i=0 ; i<meshes.size() ; i++ ) {
        MeshPtr m = meshes[i] ;
        const string &name = onames[i] ;
        scene->meshes_.push_back(m) ;
        GeometryPtr geom(new Geometry) ;
        geom->mesh_ = m ;
        NodePtr node(new Node) ;
        node->name_ = name ;
        node->geometries_.push_back(geom) ;
        scene->nodes_.push_back(node) ;
    }

    return scene ;
}

using cvx::renderer::ScenePtr ;
namespace bio = boost::iostreams ;

ScenePtr OBJSceneLoader::load_obj(const string &fname, int max_lines) const
{
    boost::filesystem::path path_(fname) ;

    if (path_.extension() == "gz" ) {
        std::ifstream ifs(fname.c_str(), std::ios::in|std::ios::binary);
        assert(ifs.good()); // XXX catch if file not found
        bio::filtering_streambuf<bio::input> in;
        in.push(bio::gzip_decompressor());
        in.push(ifs);
        std::istream istrm(&in) ;
        return load_obj_stream(istrm, fname, max_lines) ;
    }
    else {
        ifstream ifs(fname.c_str(), ios::in) ;
        return load_obj_stream(ifs, fname, max_lines) ;
    }

}
