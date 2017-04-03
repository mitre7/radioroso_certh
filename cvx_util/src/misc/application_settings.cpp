#include <cvx/util/misc/application_settings.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/detail/xml_parser_writer_settings.hpp>
#include <boost/foreach.hpp>

using namespace std ;

namespace cvx {
namespace util {

ApplicationSettings::ApplicationSettings()
{
}

bool ApplicationSettings::load(const boost::filesystem::path &fileName_)
{
    try {
        ifstream strm(fileName_.native().c_str()) ;
        boost::property_tree::xml_parser::read_xml(strm, pt_, boost::property_tree::xml_parser::trim_whitespace) ;
        file_name_ = fileName_ ;
        return true ;
    }
    catch ( const boost::property_tree::xml_parser::xml_parser_error & )
    {
        return false ;
    }
}

bool ApplicationSettings::save(const boost::filesystem::path &fileName_)
{
    using namespace boost::property_tree ;
    try {
  //      boost::property_tree::xml_writer_settings<boost::property_tree::ptree::key_type> settings('\t', 1);

        if ( fileName_.empty() )
        {
                 boost::property_tree::xml_parser::write_xml(file_name_.native(), pt_, std::locale(),
                                                              xml_parser::xml_writer_make_settings<char>('\t', 1u)) ;
        }
        else
        {
            boost::property_tree::xml_parser::write_xml(fileName_.native(), pt_, std::locale(),
                                                         xml_parser::xml_writer_make_settings<char>('\t', 1u)) ;
        }

        return true ;
    }
    catch ( const boost::property_tree::xml_parser::xml_parser_error & )
    {
        return false ;
    }
}

void ApplicationSettings::linearize(const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child, std::multimap<std::string, std::string> &ls)
{
    using boost::property_tree::ptree;

    for( ptree::const_iterator it = child.begin() ; it != child.end() ; ++it) {
        std::string data = (*it).second.get_value<std::string>() ;
        if ( !data.empty() ) ls.insert(std::pair<std::string, std::string>(childPath.dump() + '.' + (*it).first, data) )  ;
    }

    for( ptree::const_iterator it = child.begin() ; it !=child.end() ; ++it )  {
        ptree::path_type curPath = childPath / ptree::path_type(it->first);
        linearize(curPath, it->second, ls);
    }
}

string ApplicationSettings::make_key(const string &prefix) const
{
    if ( section_stack_.empty() ) return prefix ;
    else {
        string p ;
        for(int i=0 ; i<section_stack_.size() ; i++ )
        {
            p += section_stack_[i] ;
            p += '.' ;
        }
        p += prefix ;

        return p ;
    }
}

void ApplicationSettings::toMap(std::multimap<std::string, std::string> &ls)
{
    linearize("", pt_, ls) ;
}

void ApplicationSettings::keys(const std::string &prefix, std::vector<std::string> &keys) const
{
    using  boost::property_tree::ptree ;

    ptree ch = pt_.get_child(make_key(prefix)) ;

    ptree::const_iterator end = ch.end();
    for (ptree::const_iterator it = ch.begin(); it != end; it++)
    {
        std::string key = (*it).first ;
        std::string val = (*it).second.data() ;

        if ( !val.empty() ) keys.push_back(key) ;
    }
}

void ApplicationSettings::sections(const std::string &prefix, std::vector<std::string> &sections) const
{
    using  boost::property_tree::ptree ;

    ptree ch = pt_.get_child(make_key(prefix)) ;

    ptree::const_iterator end = ch.end();
    for (ptree::const_iterator it = ch.begin(); it != end; it++)
    {
        std::string key = (*it).first ;
        std::string val = (*it).second.data() ;

        if ( val.empty() ) sections.push_back(key) ;
    }
}

void ApplicationSettings::attributes(const std::string &key, std::map<std::string, std::string> &attrs)
{
    using  boost::property_tree::ptree ;

    try {
        ptree ch = pt_.get_child(make_key(key) + ".<xmlattr>") ;

        ptree::const_iterator end = ch.end();
        for (ptree::const_iterator it = ch.begin(); it != end; it++)
        {
            std::string key = (*it).first ;
            std::string val = (*it).second.data() ;

            attrs[key] = val ;
        }
    }
    catch ( boost::property_tree::ptree_bad_path &e )
    {
        return ;
    }
}


void ApplicationSettings::beginSection(const string &sec)
{
    section_stack_.push_back(sec) ;
}

void ApplicationSettings::endSection()
{
    assert(!section_stack_.empty()) ;
    section_stack_.pop_back() ;
}


} // namespace util
} // namespace cvx
