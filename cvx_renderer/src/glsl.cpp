#include "glsl.hpp"

#include <cstring>
#include <boost/format.hpp>

using namespace std ;
using namespace Eigen ;

namespace glsl {

Shader::Shader(glsl::Shader::Type t, const char *code)
{
    if ( ( handle_ = glCreateShader(t) ) == 0 )
        throw Error("cannot create shader") ;

    const GLchar* p[1];
    p[0] = code ;
    GLint lengths[1] = { (GLint)strlen(code) };

    glShaderSource(handle_, 1, p, lengths);
    glCompileShader(handle_);

    GLint success;
    glGetShaderiv(handle_, GL_COMPILE_STATUS, &success);

    if ( !success ) {
        GLchar InfoLog[1024];
        glGetShaderInfoLog(handle_, 1024, NULL, InfoLog);
        throw Error(boost::str(boost::format("Error compiling shader: '%s'\n") % InfoLog));
    }
}

Shader::~Shader() {
    glDeleteShader(handle_) ;
}

Program::Program(const char *vshader_code, const char *fshader_code)
{
    handle_ = glCreateProgram();

    ShaderPtr vertex_shader(new Shader(Shader::VERTEX, vshader_code)) ;
    ShaderPtr fragment_shader(new Shader(Shader::FRAGMENT, fshader_code)) ;

    shaders_.push_back(vertex_shader) ;
    shaders_.push_back(fragment_shader) ;

    link_and_validate_program() ;
}
/*
Program::Program(std::initializer_list<ShaderPtr> &shaders)
{
    handle_ = glCreateProgram();
    std::copy(shaders.begin(), shaders.end(), std::back_inserter(shaders_)) ;
    link_and_validate_program() ;
}
*/
void Program::setUniform(const string &name, float v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform1f(loc, v) ;
}

void Program::setUniform(const string &name, GLuint v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform1ui(loc, v) ;
}

void Program::setUniform(const string &name, GLint v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform1i(loc, v) ;
}

void Program::setUniform(const string &name, const Vector3f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform3fv(loc, 1, v.data()) ;
}

void Program::setUniform(const string &name, const Vector4f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniform4fv(loc, 1, v.data()) ;
}

void Program::setUniform(const string &name, const Matrix3f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniformMatrix3fv(loc, 1, GL_FALSE, v.data()) ;
}

void Program::setUniform(const string &name, const Matrix4f &v)
{
    GLint loc = glGetUniformLocation(handle_, name.c_str()) ;
    if ( loc != -1 ) glUniformMatrix4fv(loc, 1, GL_FALSE, v.data()) ;
}

Program::~Program() {
    glDeleteProgram(handle_) ;
}

void Program::link_and_validate_program() {

    /*

    const GLchar* feedbackVaryings[] = { "gl_Position" };
    glTransformFeedbackVaryings(id, 1, feedbackVaryings, GL_INTERLEAVED_ATTRIBS);
*/

    for( uint i=0 ; i<shaders_.size() ; i++ )
        glAttachShader(handle_, shaders_[i]->handle()) ;

    GLchar error_log[1024] = { 0 };

    // link program

    glLinkProgram(handle_) ;

    GLint success;
    glGetProgramiv(handle_, GL_LINK_STATUS, &success);

    if ( success == 0 ) {
        glGetProgramInfoLog(handle_, sizeof(error_log), NULL, error_log);
        throw Error(boost::str(boost::format("Error linking shader program: '%s'\n") % error_log));
    }

    glValidateProgram(handle_);
    glGetProgramiv(handle_, GL_VALIDATE_STATUS, &success);

    if ( !success ) {
        glGetProgramInfoLog(handle_, sizeof(error_log), NULL, error_log);
        throw Error(boost::str(boost::format("Invalid shader program: '%s'\n") % error_log));
    }
}
}
