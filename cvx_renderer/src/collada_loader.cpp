#include "collada_loader.hpp"
#include "dae_parser.hpp"
#include "minizip/unzip.h"
#include "pugixml.hpp"

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

using namespace std ;
using namespace certh_scene ;

namespace fs = boost::filesystem ;

#define dir_delimter '/'
#define MAX_FILENAME 512
#define READ_SIZE 1024
#define READ_FILE_SIZE 8192

static bool checkXml(const string &buffer, const string &token) {
    if ( !boost::starts_with(buffer, "<?xml") ) return false ;
    return ( buffer.find(token) != string::npos ) ;
}

struct ZipFile {
    ZipFile(const string &fname) {
        handle_ = unzOpen(fname.c_str()) ;
    }

    ~ZipFile() {
        if ( handle_ ) unzClose(handle_) ;
    }

    bool checkZaeFile() ;
    bool extractFile( const string &file_name, const fs::path &out_path ) ;
    bool unzip(const fs::path &root_dir) ;

    unzFile handle() const { return handle_ ; }
    unzFile handle_ ;
};

bool ZipFile::checkZaeFile()
{
    if ( !handle_ ) return false ;
    if ( unzLocateFile(handle_, "manifest.xml", 0) != UNZ_OK ) return false ;
    if ( unzOpenCurrentFile( handle_ ) != UNZ_OK ) return false ;

    int error = UNZ_OK ;
    char read_buffer[ READ_SIZE ];

    error = unzReadCurrentFile( handle_, read_buffer, READ_SIZE );
    if ( error < 0 ) {
        unzCloseCurrentFile( handle_ );
        return false ;
    }

    unzCloseCurrentFile( handle_ );

    return checkXml(read_buffer, "<dae_root") ;
}

bool ZipFile::extractFile( const string &file_name, const fs::path &out_path ) {

    if ( !handle_ ) return false ;
    if ( unzLocateFile(handle_, file_name.c_str(), 0) != UNZ_OK ) return false ;
    if ( unzOpenCurrentFile( handle_ ) != UNZ_OK ) return false ;

    char read_buffer[ READ_FILE_SIZE ];

    ofstream outf(out_path.c_str()) ;
    int nb ;

    do
    {
        nb = unzReadCurrentFile( handle_, read_buffer, READ_SIZE );
        if ( nb < 0 ) {
            unzCloseCurrentFile( handle_ );
            return false ;
        }

        if ( nb > 0 )
            outf.write(read_buffer, nb) ;
    } while ( nb > 0 );

    unzCloseCurrentFile( handle_ );

    return true ;
}

bool ZipFile::unzip(const fs::path &root_dir) {

    // Get info about the zip file

     unz_global_info global_info;

     if ( unzGetGlobalInfo( handle_, &global_info ) != UNZ_OK ) return false ;

     // Loop to extract all files
     uLong i;

     for ( i = 0; i < global_info.number_entry; ++i )
     {
         // Get info about current file.
         unz_file_info file_info;

         char buffer[ MAX_FILENAME ];

         if ( unzGetCurrentFileInfo( handle_,  &file_info, buffer, MAX_FILENAME,  NULL, 0, NULL, 0 ) != UNZ_OK )
             return false ;

         string filename(buffer) ;

         fs::path ap = root_dir / filename ;

         if ( boost::ends_with(filename, "\\") || boost::ends_with(filename, "/") )
            fs::create_directories(ap) ;
         else if ( ap.extension().native() == ".zip" || ap.extension().native() == ".zae" ) {
             // if it is a zip file we extract it to a temporary file and call unzip recursively
             fs::create_directories(ap) ;

             fs::path tmp_dir = fs::temp_directory_path() / fs::unique_path() ;

             extractFile(filename, tmp_dir) ;

             ZipFile izip(tmp_dir.native()) ;
             if ( !izip.unzip(ap) ) return false ;

             fs::remove(tmp_dir) ;
         }
         else {
             // normal file
             fs::create_directories(ap.parent_path()) ;

             extractFile(filename, ap) ;
         }


         if ( (i+1) < global_info.number_entry && unzGoToNextFile(handle_) != UNZ_OK) return false ;
      }

     return true ;
}

bool ColladaSceneLoader::canLoad(const std::string &fname) const
{
    // see if this is a zip file

    if ( ZipFile(fname).checkZaeFile() ) return true ;

    // try xml

    ifstream is(fname) ;

    char read_buffer[ READ_SIZE ];
    is.read(read_buffer, READ_SIZE) ;

    return checkXml(read_buffer, "<COLLADA") ;
}


ScenePtr ColladaSceneLoader::load(const std::string &fname)
{
    ZipFile zipfile(fname) ;

    // a zip file will be uncompressed to a temporary folder and then read

    if ( zipfile.handle() ) {

        fs::path tmp_dir = fs::temp_directory_path() / fs::unique_path() ;
        fs::create_directories(tmp_dir) ;

        if ( !zipfile.unzip(tmp_dir) )
            throw SceneLoaderException("collada", "error while uncompressing .zae file", fname) ;

        // find root dae file by reading manifest.xml

        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file( ( tmp_dir / "manifest.xml").native().c_str());

        if ( !result )
             throw SceneLoaderException("collada", "cannot parse manifest.xml in .zae file", fname) ;

        pugi::xml_node root = doc.child("dae_root") ; // descendants ?

        if ( root.empty() )
            throw SceneLoaderException("collada", "invalid manifest.xml in .zae file", fname) ;

        string uri = root.text().as_string() ;

        fs::path root_dae = tmp_dir / uri  ;

        if ( !fs::exists(root_dae) )
            throw SceneLoaderException("collada", "cannot find root dae file in .zae archive", fname) ;

        try {
            return load_dae(fs::canonical(root_dae).native()) ;
            fs::remove_all(tmp_dir);
        }
        catch ( SceneLoaderException &e ) {
           fs::remove_all(tmp_dir);
           throw e ;
        }
    }
    else
        return load_dae(fname) ;
}


ScenePtr ColladaSceneLoader::load_dae(const std::string &fname) {

    using namespace pugi ;

    cout << fname << endl ;

    ScenePtr res(new Scene) ;

    DaeParser parser(*res) ;

    if ( !parser.parse(fname) ) {
        throw SceneLoaderException("collada", "invalid XML while reading dae file", fname) ;
    }
    else return res ;
}

ColladaSceneLoader::ColladaSceneLoader() {}
