#ifndef __SCENE_IMPORTER_HPP__
#define __SCENE_IMPORTER_HPP__

#include <string>
#include <stdexcept>
#include <vector>
#include <memory>
#include <boost/lexical_cast.hpp>

#include <cvx/renderer/scene.hpp>

class SceneLoader ;
typedef boost::shared_ptr<SceneLoader> SceneLoaderPtr ;

class SceneLoader {
protected:

    friend class SceneLoaderRegistry ;

    virtual bool canLoad(const std::string &fname) const = 0 ;

public:

    // should throw exception if failed
    virtual cvx::renderer::ScenePtr load(const std::string &fname) = 0 ;

    /** Return a ';' separated list of valid file extensions for this driver. */
    virtual const char *getExtensions() const =0 ;

    /** Determines the file format from the file name extension */
    static SceneLoaderPtr findByFileName(const std::string &fname) ;

    static SceneLoaderPtr findDriver(const std::string &name) ;
} ;

class SceneLoaderRegistry
{
    SceneLoaderRegistry() ;
public:

    std::vector<SceneLoaderPtr> driver_list_ ;

    static SceneLoaderRegistry &instance()
    {
        static SceneLoaderRegistry instance_ ;

        return instance_ ;
    }
} ;

class SceneLoaderException: public std::runtime_error {

public:

    SceneLoaderException(const std::string &driver_name, const std::string &message, const std::string &fname, int line = -1):
        std::runtime_error(driver_name + ":" + message + "(" + fname + ((line > 0) ? (":" + boost::lexical_cast<std::string>(line)) : "") + ")") {}
};


#endif
