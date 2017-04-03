#include "scene_loader.hpp"

#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

//#include "collada_loader.hpp"
#include "obj_loader.hpp"

namespace fs = boost::filesystem ;
using namespace std ;


SceneLoaderRegistry::SceneLoaderRegistry() {
    driver_list_.push_back(SceneLoaderPtr(new OBJSceneLoader())) ;
}

SceneLoaderPtr SceneLoader::findDriver(const std::string &name)
{
    const vector<SceneLoaderPtr> &drivers = SceneLoaderRegistry::instance().driver_list_ ;
    for ( uint i=0 ; i<drivers.size() ; i++ )
        if ( drivers[i]->canLoad(name) ) return drivers[i] ;

    return SceneLoaderPtr() ;
}

SceneLoaderPtr SceneLoader::findByFileName(const std::string &fname)
{
    string fext = boost::filesystem::path(fname).extension().string() ;

    if ( fext.empty() ) return SceneLoaderPtr() ;

    boost::algorithm::to_lower(fext) ;

    const vector<SceneLoaderPtr> &drivers = SceneLoaderRegistry::instance().driver_list_ ;

    for ( uint i=0 ; i<drivers.size() ; i++ ) {
        SceneLoaderPtr d = drivers[i] ;

        string extensions = d->getExtensions() ;

        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep(";");

        tokenizer tokens(extensions, sep);

        tokenizer::const_iterator it = tokens.begin(), end ;
        for( ; it != end ; it++ )
            if ( *it == fext ) return d ;
    }

    return SceneLoaderPtr() ;
}
