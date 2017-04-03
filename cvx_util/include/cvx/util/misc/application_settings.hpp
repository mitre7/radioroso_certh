#ifndef __APPLICATION_SETTINGS_HPP__
#define __APPLICATION_SETTINGS_HPP__

#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem/path.hpp>
#include <map>
#include <vector>
#include <deque>

namespace cvx {
namespace util {

// provides mechanism to serialize application settings in XML config file
class ApplicationSettings
{
public:
    ApplicationSettings() ;

    virtual bool load(const boost::filesystem::path &fileName) ;
    virtual bool save(const boost::filesystem::path &fileName = boost::filesystem::path()) ;

    // get all keys corresponding to the given prefix
    void keys(const std::string &prefix, std::vector<std::string> &keys) const ;

    // get all sections corresponding to the given prefix
    void sections(const std::string &prefix, std::vector<std::string> &sections) const ;

    // get attributes of a key
    void attributes(const std::string &key, std::map<std::string, std::string> &attrs) ;

    template <class T>
    T value(const std::string &key, const T &def ) const { return pt_.get(make_key(key), def); }

    // set the value corresponding to a key with an optional attribute string.
    // key is in the form <section1>.<section2>...<key>
    // option string can be:
    // bool for boolean,
    // dec:[min,max) or dec:(min,max] or float:(min,max) etc. for numeric values
    // path:filename or path:dir or simply path for specifying that a string is a path.
    // enum:[option1, option2, option3 ..] for an enumeration. The actual value in this case can be a number or a string

    template <class T>
    void setValue(const std::string &key, const T &val, const std::string &options = std::string())  {
        pt_.put(make_key(key), val);
        if (!options.empty() ) pt_.put(key + ".<xmlattr>.options", options) ;
    }

    void toMap(std::multimap<std::string, std::string> &ls) ;

    // starts a new section. All subsequent queries will be prefixed with the section(s) key(s).

    void beginSection(const std::string &sec) ;
    void endSection() ;

private:

    void set_attribute(const std::string key, const std::string &options) ;
    void linearize(const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child, std::multimap<std::string, std::string> &ls) ;

    std::string make_key(const std::string &prefix) const ;

    boost::property_tree::ptree pt_ ;

    boost::filesystem::path file_name_ ;
    std::deque<std::string> section_stack_ ;
};

}
}

#endif
