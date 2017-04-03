#include <cvx/renderer/scene.hpp>
#include <iostream>

#include "scene_loader.hpp"

using namespace std ;

namespace cvx { namespace renderer {

ScenePtr Scene::load(const std::string &fname)
{
    SceneLoaderPtr driver = SceneLoader::findDriver(fname) ;

    if ( !driver ) return ScenePtr() ;

    try {
        return driver->load(fname) ;
    }
    catch ( SceneLoaderException &e ) {
        cerr << e.what() << endl ;
        return ScenePtr() ;
    }
}

}}
