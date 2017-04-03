#ifndef __FILESYSTEM_HPP__
#define __FILESYSTEM_HPP__

#include <boost/filesystem.hpp>

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

// various filesystem helpers

using boost::filesystem::path ;
using std::vector ;
using std::string ;

namespace cvx { namespace util {

// File system

// Find all files/dirs matching a filter
// e.g. similar to php Glob function glob("my/*/dir/*.cpp");
// pattern can contain *, ? and character ranges [], [^] as well as %0xd symbols.
// Notice that are ambiguities when backslashes are used (e.g \*) between folder separators
// and glob escapes. Also two question marks followed by slash is a trigraph and therefore should be prefixed by backslash

vector<string> glob(const string &pattern) ;

// tests if filename matches the filter. The filter is a sequence of glob patterns separated by semicolon

bool pathMatchesFilter(const path &pathName, const string &filter) ;

// get all subdirs that match the filter (sequence of glob patterns)

vector<string> subDirs(const path &dirName, const string &filter) ;

// get all files int the directory that match the filter (sequence of glob patterns)

vector<string> pathEntries(const path &dirName, const string &filter) ;

// Get temporary file path located at the specified directory. If dirName is NULL the system temp
// path is used. It does not actually create the file, so it must be used immediately

path getTemporaryPath(const string &dirName = string(), const string &prefix = string(), const string &ext = string()) ;

// platform independent paths for standard filesystem locations. On Windows they are determined by API calls.
// on Linux homepath is obtained from $HOME variable. Native datapath is $HOME/.local/share, application file path is obtained from the symlink /proc/<pid>/exe

path homePath() ;
path nativeDataDir() ;
path appFilePath() ;

// use fprintf format pattern to create filename then call cv::imwrite
void imwritef(const cv::Mat &im, const char *format, ...) ;

} /*namespace util */ }
#endif
