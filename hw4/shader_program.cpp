#include "shader_program.h"

#include <stdexcept>
#include <vector>

namespace {
std::string shader_type_name(GLenum type) {
    switch (type) {
    case GL_VERTEX_SHADER:
        return "vertex";
    case GL_FRAGMENT_SHADER:
        return "fragment";
#ifdef GL_GEOMETRY_SHADER
    case GL_GEOMETRY_SHADER:
        return "geometry";
#endif
    default:
        return "unknown";
    }
}

std::string collect_log(GLuint object, bool program) {
    GLint log_length = 0;
    if (program) {
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    } else {
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    }
    std::string log;
    log.resize(log_length > 0 ? static_cast<std::size_t>(log_length) : 1);
    if (program) {
        glGetProgramInfoLog(object, log_length, nullptr, log.data());
    } else {
        glGetShaderInfoLog(object, log_length, nullptr, log.data());
    }
    return log;
}
} // namespace

GLuint compile_shader(GLenum type, const std::string& source) {
    GLuint shader = glCreateShader(type);
    if (!shader) {
        throw std::runtime_error("Failed to create shader");
    }
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
        std::string log = collect_log(shader, false);
        glDeleteShader(shader);
        throw std::runtime_error("Failed to compile " + shader_type_name(type) + " shader: " + log);
    }
    return shader;
}

GLuint link_program(GLuint vertex_shader, GLuint fragment_shader) {
    GLuint program = glCreateProgram();
    if (!program) {
        throw std::runtime_error("Failed to create shader program");
    }
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    GLint status = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status != GL_TRUE) {
        std::string log = collect_log(program, true);
        glDeleteProgram(program);
        throw std::runtime_error("Failed to link shader program: " + log);
    }
    return program;
}

GLuint build_shader_program(const std::string& vertex_source, const std::string& fragment_source) {
    GLuint vert = compile_shader(GL_VERTEX_SHADER, vertex_source);
    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
    GLuint program = 0;
    try {
        program = link_program(vert, frag);
    } catch (...) {
        glDeleteShader(vert);
        glDeleteShader(frag);
        throw;
    }
    glDetachShader(program, vert);
    glDetachShader(program, frag);
    glDeleteShader(vert);
    glDeleteShader(frag);
    return program;
}
