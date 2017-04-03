#ifndef __OBJ_LOADER_HPP__
#define __OBJ_LOADER_HPP__

#include "scene_loader.hpp"

using cvx::renderer::ScenePtr ;

class OBJSceneLoader: public SceneLoader
{
public:
    OBJSceneLoader() ;

    ScenePtr load(const std::string &fname) ;

    const char *getExtensions() const { return "obj" ; }

protected:

    bool canLoad(const std::string &fname) const ;

private:

    ScenePtr load_obj(const std::string &fname, int max_lines = -1) const ;
    ScenePtr load_obj_stream(std::istream &strm, const std::string &fname, int max_lines) const;
};


#endif
