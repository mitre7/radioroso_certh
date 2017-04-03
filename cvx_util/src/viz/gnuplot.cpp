#include <cvx/util/viz/gnuplot.hpp>

#include <stdio.h>
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/version.hpp>

using namespace std ;
namespace fs = boost::filesystem ;

namespace cvx { namespace util {


string GnuplotPipeWrapper::gnuplot_path_ ;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
string GnuplotPipeWrapper::gnuplot_args_ = "-persist 2> NUL" ;
string GnuplotPipeWrapper::gnuplot_cmd_ = "gnuplot.exe" ;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
string GnuplotPipeWrapper::gnuplot_args_ = "-persist" ;
string GnuplotPipeWrapper::gnuplot_cmd_ = "gnuplot" ;
#endif

int GnuplotPipeWrapper::get_fileno() {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
        return _fileno(pipe_) ;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
        return fileno(pipe_) ;
#endif
}

void GnuplotPipeWrapper::close()
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
        _pclose(pipe_) ;
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
        pclose(pipe_) ;
#endif
}

bool GnuplotPipeWrapper::find_gnuplot_path()
{
    // chech if path has already been previously determined

    if ( !GnuplotPipeWrapper::gnuplot_path_.empty() &&
         fs::exists(GnuplotPipeWrapper::gnuplot_path_ + '/' + GnuplotPipeWrapper::gnuplot_cmd_) )
        return true ;

    // test if environment variable GNUPLOT_CMD_PATH is defined

    const char *epath ;
    if ( epath = getenv("GNUPLOT_CMD_PATH")  )
    {
        fs::path p(epath) ;
        p += GnuplotPipeWrapper::gnuplot_cmd_ ;

        if ( fs::exists(p) )
        {
            GnuplotPipeWrapper::gnuplot_path_ = epath ;
            return true ;
        }
    }

    if ( epath = getenv("PATH") )
    {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
        boost::char_separator<char> sep(";");
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
        boost::char_separator<char> sep(":");
#endif

        string spath(epath) ;
        boost::tokenizer<boost::char_separator<char> > tokens(spath, sep);
        boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin() ;

        BOOST_FOREACH(string t, tokens)
        {
            if ( fs::exists(t + '/' + GnuplotPipeWrapper::gnuplot_cmd_) )
            {
                GnuplotPipeWrapper::gnuplot_path_ = t ;
                return true ;
            }

        }
    }

    return false ;
}

bool GnuplotPipeWrapper::init()
{
    if ( !find_gnuplot_path() ) return false ;

    // open pipe

    std::string cmd = GnuplotPipeWrapper::gnuplot_path_ + "/" + GnuplotPipeWrapper::gnuplot_cmd_ +
            ' ' + GnuplotPipeWrapper::gnuplot_args_ ;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
    pipe_ = _popen(cmd.c_str(),"w");
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
    pipe_ = popen(cmd.c_str(), "w") ;
#endif

    if ( !pipe_ ) return false ;

    return true ;

}


Gnuplot::Gnuplot(): GnuplotPipeWrapper(),
    boost::iostreams::stream<boost::iostreams::file_descriptor_sink>( get_fileno(),
#if BOOST_VERSION >= 104400
    boost::iostreams::never_close_handle
#else
   false
#endif
)
{
    assert( pipe_ ) ;
}

Gnuplot::Gnuplot(const string &outFile): GnuplotPipeWrapper(),
    boost::iostreams::stream<boost::iostreams::file_descriptor_sink>( get_fileno(),
#if BOOST_VERSION >= 104400
    boost::iostreams::never_close_handle
#else
   false
#endif
                                                                      )
{
    fs::path epath(outFile) ;

    string ext = epath.extension().string() ;

    if ( ext == ".eps" )
        *this << "set terminal epslatex" << endl ;
    else if ( ext == ".pdf" )
        *this << "set terminal pdfcairo" << endl ;
    else if ( ext == ".png" )
        *this << "set terminal pngcairo" << endl ;

    *this << "set output \"" << outFile << "\"" << endl ;
}


Gnuplot::~Gnuplot()
{
    *this << "set output" << endl ;
    *this << "set terminal pop" << endl ;

    GnuplotPipeWrapper::close() ;

    for(int i=0 ; i<tmp_files_.size() ; i++)
    {
        if ( fs::exists(tmp_files_[i]) )
            fs::remove(tmp_files_[i]) ;
    }

}

string Gnuplot::make_tmp_file()
{
    fs::path p = boost::filesystem::unique_path(fs::temp_directory_path() / "tmp-gnuplot-%%%%%") ;

    string tmp_file_path = p.string() ;
    tmp_files_.push_back(tmp_file_path) ;

    return tmp_file_path ;
}


}
}
