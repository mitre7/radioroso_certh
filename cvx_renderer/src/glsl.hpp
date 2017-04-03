#ifndef GLSL_HPP
#define GLSL_HPP

#include <GL/glew.h>
#include <string>
#include <stdexcept>
#include <memory>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace glsl {

class Shader {
public:
    enum Type { VERTEX = GL_VERTEX_SHADER,  FRAGMENT = GL_FRAGMENT_SHADER, GEOMETRY = GL_GEOMETRY_SHADER,
                COMPUTE = GL_COMPUTE_SHADER, TESS_CONTROL = GL_TESS_CONTROL_SHADER, TESS_EVALUATION = GL_TESS_EVALUATION_SHADER
              } ;

    Shader(Type t, const char *code) ;
    ~Shader() ;
    GLuint handle() const { return handle_; }

private:
    GLuint handle_ ;
};

typedef boost::shared_ptr<Shader> ShaderPtr ;

class Program {
public:

    Program(const char *vshader, const char *fshader) ;
//    Program(std::initializer_list<ShaderPtr> &shaders) ;

    void setUniform(const std::string &name, float v) ;
    void setUniform(const std::string &name, GLint v) ;
    void setUniform(const std::string &name, GLuint v) ;
    void setUniform(const std::string &name, const Eigen::Vector3f &v) ;
    void setUniform(const std::string &name, const Eigen::Vector4f &v) ;
    void setUniform(const std::string &name, const Eigen::Matrix3f &v) ;
    void setUniform(const std::string &name, const Eigen::Matrix4f &v) ;

    void use() { glUseProgram(handle_) ; }

    ~Program() ;

private:

    void link_and_validate_program() ;

    GLuint handle_ ;
    std::vector<ShaderPtr> shaders_ ;
};

typedef boost::shared_ptr<Program> ProgramPtr ;

class Error: public std::runtime_error {
public:
    Error(const std::string &msg):
        std::runtime_error(msg), id_(glGetError()) {}

    GLenum glError() const { return id_ ; }

private:
    GLenum id_ ;
};

}

#endif // GLSL_HPP
