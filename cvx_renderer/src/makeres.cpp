/* http://hamelot.co.uk/visualization/opengl-glsl-shader-as-a-string/ */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::filesystem ;

std::string make_c_string( const string & in) {
    string out;
    for (size_t i = 0; i < in.size(); ++i) {
        char c = in[i];
        if ('"' == c)
            out += "\\\"";
        else if ('\\' == c)
            out += "\\\\";
        else
            out += c;
    }
    return out ;
}

void write_quoted_string(ostream &strm, const string &str) {
    strm << "\"" << str << "\"";
}

int main(int argc, char** args) {
    if (argc < 4) {
        printf("syntax error, usage:  makeres array_postfix infile outfile");
        exit(0xff);
    }

    string postfix = args[1];
    string in_file = args[2] ;
    string out_file = args[3] ;

    path p(in_file) ;
    string fname = p.stem().string() ;
    string fext = p.extension().string(), type ;
    if ( fext == ".vs") type = "vertex_shader" ;
    else if ( fext == ".fs" ) type = "fragment_shader" ;
    else type == "unknown" ;

    string array_name = fname + "_" + type + postfix ;

    ofstream ostrm(out_file.c_str()) ;

    string uc = boost::to_upper_copy(fname) ;
    ostrm << "#ifndef  SHADER_HEADER_" << uc << endl ;
    ostrm << "#define  SHADER_HEADER_" << uc << endl ;
    ostrm << endl ;

    ostrm << "const char *" << array_name << "=" << endl ;

    ifstream istrm(in_file.c_str()) ;

    bool first = true ;

    while ( istrm ) {
        string line ;
        std::getline(istrm, line) ;

        if ( !first ) ostrm << endl ;
        first = false ;

        write_quoted_string(ostrm, make_c_string(line.substr(0, line.find("\n"))) + "\\n") ;
    }

    ostrm << ';' << endl ;
    ostrm << "#endif // SHADER_HEADER_" << uc ;
}
