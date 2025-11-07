#ifndef HW4_SHADER_PROGRAM_H
#define HW4_SHADER_PROGRAM_H

#include <GL/glew.h>
#include <string>

GLuint compile_shader(GLenum type, const std::string& source);
GLuint link_program(GLuint vertex_shader, GLuint fragment_shader);
GLuint build_shader_program(const std::string& vertex_source, const std::string& fragment_source);

#endif
