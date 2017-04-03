#ifndef __COLLADA_LOADER_HPP__
#define __COLLADA_LOADER_HPP__

#include "scene_loader.hpp"

using certh_scene::ScenePtr ;

class ColladaSceneLoader: public SceneLoader
{
public:
    ColladaSceneLoader() ;

    ScenePtr load(const std::string &fname) ;

    const char *getExtensions() const { return "dae;zae" ; }

protected:

    bool canLoad(const std::string &fname) const ;

private:

    ScenePtr load_dae(const std::string &fname) ;
};


#endif
