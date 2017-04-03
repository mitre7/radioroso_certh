#include <cvx/util/misc/filesystem.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <cstdarg>

#ifdef  _WIN32
#include <windows.h>
#include <rpc.h>
//#pragma comment(lib, "Rpcrt4.lib") // For using UUID generator
#endif

using namespace std ;
using namespace boost::filesystem ;
using namespace boost ;

typedef boost::tokenizer<boost::char_separator<char> > stokenizer;

namespace cvx { namespace util {

static string globToRegex(const char *pat)
{
    // Convert pattern
    string rx = "(?i)^", be ;

    int i = 0;
    const char *pc = pat ;
    int clen = strlen(pat) ;
    bool in_char_class = false ;

    while (i < clen)
    {
        char c = pc[i++];

        switch (c)
        {
            case '*':
                rx += "[^\\\\/]*" ;
                break;
            case '?':
                rx += "[^\\\\/]" ;
                break;
            case '$':  //Regex special characters
            case '(':
            case ')':
            case '+':
            case '.':
            case '|':
                rx += '\\';
                rx += c;
                break;
            case '\\':
                if ( pc[i] == '*' ) rx += "\\*" ;
                else if ( pc[i] == '?' )  rx += "\\?" ;
                ++i ;
            break ;
            case '[':
            {
                if ( in_char_class )  rx += "\\[";
                else {
                    in_char_class = true ;
                    be += c ;
                }
                break ;
            }
            case ']':
            {
                if ( in_char_class ) {
                    in_char_class = false ;
                    rx += be ;
                    rx += c ;
                    rx += "{1}" ;
                    be.clear() ;
                }
                else rx += "\\]" ;

                break ;
            }
            case '%':
            {
                regex rd("(0\\d)?d") ;
                smatch what;

                if ( regex_match(std::string(pat + i), what, rd,  boost::match_extra) )
                {

                    rx += "[[:digit:]]" ;

                    if ( what.size() == 2 )
                    {
                        rx +=  "{" ;
                        rx += what[1] ;
                        rx += "}" ;
                    }
                    else
                        rx += "+" ;

                    i += what.size() ;
                }
                else
                {
                    if ( in_char_class ) be += c ;
                    else rx += c;
                }
                break ;

            }
            default:
                if ( in_char_class ) be += c ;
                else rx += c;
        }
    }

    rx += "$" ;
    return rx ;
}

static void globFindSubDirs(const string &dir, int i, std::vector<string> &pats, std::vector<string> &files, bool trailing_slash)
{
    if ( i > pats.size()-1  ) { // we reached the last element
        if ( is_directory(path(dir))  )
            files.push_back(path(dir + '/').generic_string()) ;
        else if ( !trailing_slash )
            files.push_back(path(dir).generic_string()) ;
        return ;
    }

    regex rm(globToRegex(pats[i].c_str())) ;

    boost::system::error_code ec;
    directory_iterator it(dir, ec) ;

    while (it != directory_iterator())
    {
        path p = (*it).path() ;
        path subpath = p.leaf() ;

        string subpaths = subpath.string() ;

        if ( regex_match(subpaths, rm) )
             globFindSubDirs(p.string(), i+1, pats, files, trailing_slash) ;

        it.increment(ec) ;
        if ( ec) ;
     }
}

static regex driverx("(?i)^([a-z]:(?:\\\\|/)|(?:\\\\\\\\|//)[^~!@#$^&()=+\\[\\]{}\\\\|;:',<>/?]+(?:\\\\|/)[^~!@#$^&()=+\\[\\]{}\\\\|;:',<>/?]+(?:[\\\\/])|(?:[\\\\/]))") ;

std::vector<string> glob(const std::string &pattern)
{
    // Find and erase any drive specification

    int matchlen = 0;
    int pos = -1 ;

    smatch what ;
    string dir(pattern) ;
    string drive = "/" ;

    if ( regex_search(dir, what, driverx, boost::match_extra ) )
    {
        pos = 0;

        matchlen = what[0].length() ;

        drive = what[0] ;
        dir = what.suffix() ;
        if ( drive.empty() ) drive = absolute(path(".")).string();
    }
    std::vector<string> el ;

    char_separator<char> sep("/\\", 0, boost::keep_empty_tokens);

    stokenizer tokens(dir, sep);

    std::vector<string> subdirs ;

    bool trailing_slash = false ;

    for (stokenizer::iterator tok_iter = tokens.begin();
         tok_iter != tokens.end(); ++tok_iter )
    {
        string el = *tok_iter ;

        if ( el == "." ) continue ;
        else if ( el == ".." ) {
            if ( subdirs.empty() ) drive += "../" ;
            else subdirs.pop_back() ;
        }
        else if ( el.empty() ) { trailing_slash = true ; continue ; }
        else {
            subdirs.push_back(el) ;
            trailing_slash = false ;
        }
    }

    vector<string> fnames ;
    globFindSubDirs(drive, 0, subdirs, fnames, trailing_slash) ;

    return fnames ;
}

bool pathMatchesFilter(const boost::filesystem::path &pathName, const std::string &filter)
{
    if ( filter.empty() ) return true ;

    std::string sf(filter) ;
    std::vector<string> el ;

    algorithm::split( el, sf, is_any_of(";"), algorithm::token_compress_on );

    for(int i=0 ; i<el.size() ; i++ )
    {
        regex rm(globToRegex(el[i].c_str())) ;

        if ( regex_match(pathName.string(), rm) ) return true ;
    }

    return false ;
}

// get all subdirs that match the filter (sequence of glob patterns)
std::vector<string> subDirs(const boost::filesystem::path &dirName, const std::string &filter)
{
    vector<string> dirs ;

    if ( filter.empty() ) return dirs ;

    std::string sf(filter) ;
    std::vector<string> el ;

    algorithm::split( el, sf, is_any_of(";"), algorithm::token_compress_on );

    boost::system::error_code ec;
    directory_iterator it(dirName, ec), dend ;

    std::vector<regex> pats ;
    for(int i=0 ; i<el.size() ; i++)
        pats.push_back(regex(globToRegex(el[i].c_str()))) ;

    for ( ; it != dend ; ++it )
    {
        path p = (*it) ;
        path pl = p.leaf() ;

        if ( !is_directory(p) ) continue ;

        if ( pats.empty() ) dirs.push_back(pl.string()) ;
        else {
            for(int i=0 ; i<pats.size() ; i++ )
            {
                if ( regex_match(pl.string(), pats[i]) ) { dirs.push_back(pl.string()) ; }
            }
        }

    }

    return dirs ;

}

// get all files in the directory that match the filter (sequence of glob patterns)

std::vector<string> pathEntries(const boost::filesystem::path &pathName, const std::string &filter)
{
    std::vector<string> files ;

    std::string sf(filter) ;
    std::vector<string> el ;

    algorithm::split( el, sf, is_any_of(";"), algorithm::token_compress_on );

    std::vector<regex> pats ;
    for(int i=0 ; i<el.size() ; i++)
        pats.push_back(regex(globToRegex(el[i].c_str()))) ;

    boost::system::error_code ec;
    directory_iterator it(pathName, ec), dend ;

    for ( ; it != dend ; ++it )
    {
        path p = (*it) ;
        path pl = p.leaf() ;

        if ( is_directory(p) ) continue ;

        if ( pats.empty() ) files.push_back(pl.string()) ;
        else {
            for(int i=0 ; i<pats.size() ; i++ )
            {
                if ( regex_match(pl.string(), pats[i]) ) { files.push_back(pl.string()) ; }
            }
        }

    }

    return files ;
}

namespace fs = boost::filesystem ;

fs::path getTemporaryPath(const std::string &dir, const std::string &prefix, const std::string &ext)
{
    std::string retVal ;

    fs::path directory ;

    if ( ! dir.empty() ) directory = dir;
    else directory = boost::filesystem::temp_directory_path() ;

    std::string varname ="%%%%-%%%%-%%%%-%%%%";

    if ( !prefix.empty() )
        directory /= prefix + '-' + varname + '.' + ext ;
    else
        directory /= "tmp-" + varname + '.' + ext ;

    boost::filesystem::path temp = boost::filesystem::unique_path(directory);

    return temp;
}


fs::path homePath() {
#if _WIN32
    typedef BOOL (* PtrGetUserProfileDirectoryW)(HANDLE, LPWSTR, LPDWORD) ;

    PtrGetUserProfileDirectoryW ptrGetUserProfileDirectoryW ;

    HINSTANCE handle = ::LoadLibrary("userenv");

    if ( handle )
    {
        ptrGetUserProfileDirectoryW = (PtrGetUserProfileDirectoryW )::GetProcAddress(handle, "GetUserProfileDirectoryW");

        if (ptrGetUserProfileDirectoryW) {
            HANDLE hnd = ::GetCurrentProcess();
            HANDLE token = 0;
            BOOL ok = ::OpenProcessToken(hnd, TOKEN_QUERY, &token);
            if (ok) {
                DWORD dwBufferSize = 0;
                // First call, to determine size of the strings (with '\0').
                ok = ptrGetUserProfileDirectoryW(token, NULL, &dwBufferSize);
                if (!ok && dwBufferSize != 0) { // We got the required buffer size
                    wchar_t *userDirectory = new wchar_t[dwBufferSize];
                    // Second call, now we can fill the allocated buffer.
                    ok = ptrGetUserProfileDirectoryW(token, userDirectory, &dwBufferSize);
                    if (ok)
                    {
                        fs::path res(userDirectory) ;

                        return res ;
                        delete [] userDirectory;
                    }
                }
            }
            ::CloseHandle(token);
        }
     }

    return fs::path() ;


#else
    const char *home_dir = getenv("HOME") ;

    if ( home_dir ) return fs::path(home_dir) ;
    else return fs::path() ;
#endif

}



fs::path nativeDataDir()
{
#if _WIN32
    HINSTANCE handle = ::LoadLibrary("shell32");

    if ( handle )
    {
        typedef BOOL (WINAPI * GetSpecialFolderPathW)(HWND, LPWSTR, int, BOOL);

        GetSpecialFolderPathW ptrGetSpecialFolderPathW ;

        ptrGetSpecialFolderPathW = (GetSpecialFolderPathW)::GetProcAddress(handle, "SHGetSpecialFolderPathW");

        if ( !ptrGetSpecialFolderPathW ) return fs::path();

        wchar_t path[MAX_PATH];

        if (ptrGetSpecialFolderPathW(0, path, CSIDL_APPDATA, FALSE))
            return fs::path(path) ;

    }
    else return fs::path() ;
#else
     return homePath() / ".local/share/";
#endif

}


fs::path appFilePath()
{
#if _WIN32
    // We do MAX_PATH + 2 here, and request with MAX_PATH + 1, so we can handle all paths
    // up to, and including MAX_PATH size perfectly fine with string termination, as well
    // as easily detect if the file path is indeed larger than MAX_PATH, in which case we
    // need to use the heap instead. This is a work-around, since contrary to what the
    // MSDN documentation states, GetModuleFileName sometimes doesn't set the
    // ERROR_INSUFFICIENT_BUFFER error number, and we thus cannot rely on this value if
    // GetModuleFileName(0, buffer, MAX_PATH) == MAX_PATH.
    // GetModuleFileName(0, buffer, MAX_PATH + 1) == MAX_PATH just means we hit the normal
    // file path limit, and we handle it normally, if the result is MAX_PATH + 1, we use
    // heap (even if the result _might_ be exactly MAX_PATH + 1, but that's ok).
    wchar_t buffer[MAX_PATH + 2];
    DWORD v = ::GetModuleFileNameW(0, buffer, MAX_PATH + 1);
    buffer[MAX_PATH + 1] = 0;
    if (v == 0) return string();
    else if (v <= MAX_PATH)
    return fs::path(buffer);
    // MAX_PATH sized buffer wasn't large enough to contain the full path, use heap
    wchar_t *b = 0;
    int i = 1;
    size_t size;
    do {
        ++i;
        size = MAX_PATH * i;
        b = reinterpret_cast<wchar_t *>(realloc(b, (size + 1) * sizeof(wchar_t)));
        if (b)
        v = ::GetModuleFileNameW(NULL, b, size);
    } while (b && v == size);

    if (b)
    *(b + size) = 0;
    fs::path res(b) ;
    free(b);

    return res ;

#else // Linux

    stringstream fpaths ;
    fpaths << "/proc/" << getpid() << "/exe" ;
    fs::path fpath(fpaths.str()) ;

    if ( fs::exists(fpath) ) return fs::canonical(fpath).parent_path() ;
    else return fs::path() ;

#endif
}

void imwritef(const cv::Mat &im, const char *format, ...)
{
    va_list vl;
    va_start(vl, format);
    int nc = vsnprintf(0, 0, format, vl) ;
    va_end(vl) ;

    va_start(vl, format);
    char *buffer = new char [nc+1] ;
    vsnprintf(buffer, nc+1, format, vl) ;
    va_end(vl) ;

    cv::imwrite(buffer, im) ;

    delete [] buffer ;

}

} // namespace util
}
