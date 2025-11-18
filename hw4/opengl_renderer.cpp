#include "arcball.h"
#include "scene_loader.h"
#include "texture_loader.h"

#ifdef __APPLE__
// #define GL_SILENCE_DEPRECATION
#endif
#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef SHADER_DIR
#define SHADER_DIR "./shaders"
#endif

namespace {

constexpr int kMaxLights = 8;

enum class RunMode {
    Scene,
    NormalMap
};

struct Mesh {
    GLuint vao = 0;
    GLuint vbo = 0;
    GLsizei vertex_count = 0;
    Eigen::Vector3f ambient = Eigen::Vector3f::Zero();
    Eigen::Vector3f diffuse = Eigen::Vector3f::Zero();
    Eigen::Vector3f specular = Eigen::Vector3f::Zero();
    float shininess = 32.0f;
};

struct LightState {
    Eigen::Vector3f position;
    Eigen::Vector3f color;
    float attenuation = 0.0f;
};

struct SceneState {
    Scene scene;
    std::vector<Mesh> meshes;
    std::vector<LightState> lights;
};

struct SceneUniforms {
    GLint model_view = -1;
    GLint projection = -1;
    GLint normal_matrix = -1;
    GLint ambient_light = -1;
    GLint light_count = -1;
    GLint light_positions = -1;
    GLint light_colors = -1;
    GLint light_atten = -1;
    GLint material_ambient = -1;
    GLint material_diffuse = -1;
    GLint material_specular = -1;
    GLint material_shininess = -1;
    GLint shading_mode = -1;
};

struct QuadUniforms {
    GLint model_view = -1;
    GLint projection = -1;
    GLint normal_matrix = -1;
    GLint color_texture = -1;
    GLint normal_texture = -1;
    GLint light_position = -1;
    GLint light_color = -1;
    GLint ambient_light = -1;
    GLint specular = -1;
    GLint shininess = -1;
};

struct QuadState {
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    GLuint color_tex = 0;
    GLuint normal_tex = 0;
    GLsizei index_count = 0;
};

RunMode g_mode = RunMode::Scene;
SceneState g_scene_state;
QuadState g_quad_state;
Arcball g_arcball;

int g_window_width = 800;
int g_window_height = 800;
int g_shading_mode = 0; // 0 = Gouraud, 1 = Phong

Eigen::Vector3f g_ambient_light(0.1f, 0.1f, 0.1f);

GLuint g_scene_program = 0;
SceneUniforms g_scene_uniforms;

GLuint g_quad_program = 0;
QuadUniforms g_quad_uniforms;

std::string g_shader_dir = SHADER_DIR;
std::string g_scene_path;
std::string g_color_path;
std::string g_normal_path;

} // namespace

// ####################
//  Utility helpers
// #####################

std::string load_text_file(const std::string& path) {
    std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open shader file: " + path);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

GLuint compile_shader(GLenum type, const std::string& path) {
    // Type specifies vertex vs fragment shader
    const std::string source = load_text_file(path);
    const char* source_ptr = source.c_str();

    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source_ptr, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    // If compiling failed, grab and print the log
    if (ok != GL_TRUE) {
        GLint log_len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_len);
        std::vector<char> log(log_len + 1, '\0');
        glGetShaderInfoLog(shader, log_len, nullptr, log.data());
        std::string stage = (type == GL_VERTEX_SHADER) ? "vertex" : "fragment";
        glDeleteShader(shader);
        throw std::runtime_error("Failed to compile " + stage + " shader (" + path + "):\n" + log.data());
    }
    return shader;
}

GLuint link_program(GLuint vs, GLuint fs, const std::vector<std::pair<GLuint, const char*>>& attribs) {
    // attrib is a vector of pairs of assigned integers and names
    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    for (const auto& attr : attribs) {
        glBindAttribLocation(program, attr.first, attr.second);
    }
    glLinkProgram(program);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        GLint log_len = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_len);
        std::vector<char> log(log_len + 1, '\0');
        glGetProgramInfoLog(program, log_len, nullptr, log.data());
        throw std::runtime_error(std::string("Failed to link shader program:\n") + log.data());
    }
    return program;
}

Eigen::Matrix4f to_matrix4f(const Eigen::Matrix4d& mat_d) {
    return mat_d.cast<float>();
}

Eigen::Matrix4f make_perspective(float fov_y_degrees, float aspect, float znear, float zfar) {
    // Derive parameters needed to make perspective
    const double n = static_cast<double>(znear);
    const double f = static_cast<double>(zfar);
    const double fovy_rad = static_cast<double>(fov_y_degrees) * M_PI / 180.0;
    const double t = n * std::tan(fovy_rad * 0.5);
    const double b = -t;
    const double r = t * static_cast<double>(aspect);
    const double l = -r;

    // Same as hw2 implementation
    Eigen::Matrix4d Pd;
    Pd <<
        (2.0 * n) / (r - l),  0.0,                 (r + l) / (r - l),   0.0,
        0.0,                  (2.0 * n) / (t - b), (t + b) / (t - b),   0.0,
        0.0,                  0.0,                 -(f + n) / (f - n),  -(2.0 * f * n) / (f - n),
        0.0,                  0.0,                 -1.0,                0.0;

    return Pd.cast<float>();
}

Eigen::Matrix4f make_translation(float x, float y, float z) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0,3) = x;
    mat(1,3) = y;
    mat(2,3) = z;
    return mat;
}

// #####################
//  Scene mode setup
// #####################

void build_scene_meshes() {
    g_scene_state.meshes.clear();
    g_scene_state.meshes.reserve(g_scene_state.scene.scene_objects.size());

    for (const auto& inst : g_scene_state.scene.scene_objects) {
        std::vector<float> interleaved; // interleaved vectors and normals
        interleaved.reserve(inst.obj.faces.size() * 3 * 6);

        for (const auto& face : inst.obj.faces) {
            const Vertex& v1 = inst.obj.vertices.at(face.v1);
            const Vertex& v2 = inst.obj.vertices.at(face.v2);
            const Vertex& v3 = inst.obj.vertices.at(face.v3);
            const Normal& n1 = inst.obj.normals.at(face.vn1);
            const Normal& n2 = inst.obj.normals.at(face.vn2);
            const Normal& n3 = inst.obj.normals.at(face.vn3);

            const Vertex* vertices[3] = {&v1, &v2, &v3};
            const Normal* normals[3] = {&n1, &n2, &n3};
            for (int i = 0; i < 3; ++i) {
                interleaved.push_back(static_cast<float>(vertices[i]->x));
                interleaved.push_back(static_cast<float>(vertices[i]->y));
                interleaved.push_back(static_cast<float>(vertices[i]->z));
                interleaved.push_back(static_cast<float>(normals[i]->x));
                interleaved.push_back(static_cast<float>(normals[i]->y));
                interleaved.push_back(static_cast<float>(normals[i]->z));
            }
        }

        Mesh mesh;
        mesh.vertex_count = static_cast<GLsizei>(interleaved.size() / 6);
        mesh.ambient = inst.ambient.cast<float>();
        mesh.diffuse = inst.diffuse.cast<float>();
        mesh.specular = inst.specular.cast<float>();
        mesh.shininess = static_cast<float>(std::max(0.0, std::min(inst.shininess, 200.0)));

        // Generate a vao location for this mesh
        glGenVertexArrays(1, &mesh.vao);
        glBindVertexArray(mesh.vao);

        // Store the mesh in a vbo
        glGenBuffers(1, &mesh.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)),
                     interleaved.data(), GL_STATIC_DRAW);

        // Tell information needed to interpret our vbo
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));

        // Reset so we don't accidentally overwrite
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        g_scene_state.meshes.push_back(mesh);
    }
}

void init_scene_lights() {
    g_scene_state.lights.clear();
    for (const auto& light : g_scene_state.scene.lights) {
        LightState state;
        state.position = Eigen::Vector3f(static_cast<float>(light.x), static_cast<float>(light.y), static_cast<float>(light.z));
        state.color = Eigen::Vector3f(static_cast<float>(light.r), static_cast<float>(light.g), static_cast<float>(light.b));
        state.attenuation = static_cast<float>(light.atten);
        g_scene_state.lights.push_back(state);
    }
}

void create_scene_program() {
    const std::string vertex_path = g_shader_dir + "/scene.vert";
    const std::string fragment_path = g_shader_dir + "/scene.frag";

    GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex_path);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment_path);
    g_scene_program = link_program(vs, fs, {{0, "aPosition"}, {1, "aNormal"}});
    glDeleteShader(vs);
    glDeleteShader(fs);

    g_scene_uniforms.model_view = glGetUniformLocation(g_scene_program, "uModelView");
    g_scene_uniforms.projection = glGetUniformLocation(g_scene_program, "uProjection");
    g_scene_uniforms.normal_matrix = glGetUniformLocation(g_scene_program, "uNormalMatrix");
    g_scene_uniforms.ambient_light = glGetUniformLocation(g_scene_program, "uAmbientLight");
    g_scene_uniforms.light_count = glGetUniformLocation(g_scene_program, "uLightCount");
    g_scene_uniforms.light_positions = glGetUniformLocation(g_scene_program, "uLightPositions");
    g_scene_uniforms.light_colors = glGetUniformLocation(g_scene_program, "uLightColors");
    g_scene_uniforms.light_atten = glGetUniformLocation(g_scene_program, "uLightAttenuations");
    g_scene_uniforms.material_ambient = glGetUniformLocation(g_scene_program, "uMaterialAmbient");
    g_scene_uniforms.material_diffuse = glGetUniformLocation(g_scene_program, "uMaterialDiffuse");
    g_scene_uniforms.material_specular = glGetUniformLocation(g_scene_program, "uMaterialSpecular");
    g_scene_uniforms.material_shininess = glGetUniformLocation(g_scene_program, "uMaterialShininess");
    g_scene_uniforms.shading_mode = glGetUniformLocation(g_scene_program, "uShadingMode");
}

// #####################
//  PNG Mode Setup
// #####################

void create_quad_program() {
    const std::string vertex_path = g_shader_dir + "/quad.vert";
    const std::string fragment_path = g_shader_dir + "/quad.frag";

    GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex_path);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment_path);
    g_quad_program = link_program(vs, fs, {{0, "aPosition"}, {1, "aNormal"}, {2, "aTangent"}, {3, "aBitangent"}, {4, "aTexCoord"}});
    glDeleteShader(vs);
    glDeleteShader(fs);

    g_quad_uniforms.model_view = glGetUniformLocation(g_quad_program, "uModelView");
    g_quad_uniforms.projection = glGetUniformLocation(g_quad_program, "uProjection");
    g_quad_uniforms.normal_matrix = glGetUniformLocation(g_quad_program, "uNormalMatrix");
    g_quad_uniforms.color_texture = glGetUniformLocation(g_quad_program, "uColorTexture");
    g_quad_uniforms.normal_texture = glGetUniformLocation(g_quad_program, "uNormalTexture");
    g_quad_uniforms.light_position = glGetUniformLocation(g_quad_program, "uLightPosition");
    g_quad_uniforms.light_color = glGetUniformLocation(g_quad_program, "uLightColor");
    g_quad_uniforms.ambient_light = glGetUniformLocation(g_quad_program, "uAmbientLight");
    g_quad_uniforms.specular = glGetUniformLocation(g_quad_program, "uSpecularColor");
    g_quad_uniforms.shininess = glGetUniformLocation(g_quad_program, "uShininess");
}

void build_quad_geometry() {
    struct VertexData {
        float px, py, pz;
        float nx, ny, nz;
        float tx, ty, tz;
        float bx, by, bz;
        float u, v;
    };

    // Manually specify our rectangle
    // Ordering is px py pz,    nx ny nz,   tx ty tz,   bx by bz,   u v
    const VertexData vertices[] = {
        { -1.f, -1.f, 0.f,  0.f,0.f,1.f,    1.f,0.f,0.f,    0.f,1.f,0.f,    0.f,0.f },
        {  1.f, -1.f, 0.f,  0.f,0.f,1.f,    1.f,0.f,0.f,    0.f,1.f,0.f,    1.f,0.f },
        {  1.f, 1.f, 0.f,   0.f,0.f,1.f,    1.f,0.f,0.f,    0.f,1.f,0.f,    1.f,1.f },
        { -1.f, 1.f, 0.f,   0.f,0.f,1.f,    1.f,0.f,0.f,    0.f,1.f,0.f,    0.f,1.f },
    };

    // Draw two triangles
    const unsigned int indices[] = {0, 1, 2, 0, 2, 3};

    glGenVertexArrays(1, &g_quad_state.vao);
    glBindVertexArray(g_quad_state.vao);

    glGenBuffers(1, &g_quad_state.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g_quad_state.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &g_quad_state.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_quad_state.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    const GLsizei stride = sizeof(VertexData);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(9 * sizeof(float)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(12 * sizeof(float)));

    glBindVertexArray(0);
    g_quad_state.index_count = 6;
}

// #####################
//  Rendering Helpers
// #####################

Eigen::Matrix4f compute_scene_model_view() {
    const Eigen::Matrix4d arcball_data = g_arcball.rotation().to_matrix();
    Eigen::Matrix4f arcball = to_matrix4f(arcball_data);
    // raw pointer to the Cinv doubles
    const double* raw = g_scene_state.scene.cam_transforms.Cinv.data();
    // view that memory as a 4x4 double matrix (column major)
    Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::ColMajor>> Cinv_map(raw);
    // cast to float
    Eigen::Matrix4f camera = Cinv_map.cast<float>();
    return camera * arcball;
}

Eigen::Matrix4f compute_scene_projection() {
    // raw pointer to the doubles
    const double* raw = g_scene_state.scene.cam_transforms.P.data();
    // reinterpret that memory as a 4×4 double matrix
    Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::ColMajor>> P_map(raw);
    // convert to float and return
    return P_map.cast<float>();
}

void upload_scene_globals(const Eigen::Matrix4f& model_view, const Eigen::Matrix4f& projection) {
    // Upload camera, lighting, etc once per frame
    // Transform for normals: (MV_3x3)^{-T}
    Eigen::Matrix3f normal_matrix = model_view.block<3,3>(0,0).inverse().transpose();

    // Camera transforms
    glUniformMatrix4fv(g_scene_uniforms.model_view, 1, GL_FALSE, model_view.data());
    glUniformMatrix4fv(g_scene_uniforms.projection, 1, GL_FALSE, projection.data());
    glUniformMatrix3fv(g_scene_uniforms.normal_matrix, 1, GL_FALSE, normal_matrix.data());

    // Ambient light (scene constant)
    glUniform3fv(g_scene_uniforms.ambient_light, 1, g_ambient_light.data());

    // Pack light array (clamped to kMaxLights)
    GLint light_count = static_cast<GLint>(std::min<std::size_t>(g_scene_state.lights.size(), kMaxLights));
    glUniform1i(g_scene_uniforms.light_count, light_count);

    std::vector<GLfloat> positions(kMaxLights * 3, 0.0f);
    std::vector<GLfloat> colors(kMaxLights * 3, 0.0f);
    std::vector<GLfloat> atten(kMaxLights,0.0f);

    for (GLint i = 0; i < light_count; ++i) {
        positions[i * 3 + 0] = g_scene_state.lights[i].position.x();
        positions[i * 3 + 1] = g_scene_state.lights[i].position.y();
        positions[i * 3 + 2] = g_scene_state.lights[i].position.z();

        colors[i * 3 + 0] = g_scene_state.lights[i].color.x();
        colors[i * 3 + 1] = g_scene_state.lights[i].color.y();
        colors[i * 3 + 2] = g_scene_state.lights[i].color.z();

        atten[i] = g_scene_state.lights[i].attenuation;
    }

    // Upload light arrays (shader expects arrays of vec3 and float)
    glUniform3fv(g_scene_uniforms.light_positions, light_count, positions.data());
    glUniform3fv(g_scene_uniforms.light_colors, light_count, colors.data());
    glUniform1fv(g_scene_uniforms.light_atten, light_count, atten.data());

    // Shading mode toggle
    glUniform1i(g_scene_uniforms.shading_mode, g_shading_mode);
}

void render_scene_mode() {
    // Each frame: set globals. Per-mesh: set material, bind VAO, draw.
    glUseProgram(g_scene_program);

    Eigen::Matrix4f model_view = compute_scene_model_view();
    Eigen::Matrix4f projection = compute_scene_projection();
    upload_scene_globals(model_view, projection);

    for (const auto& mesh : g_scene_state.meshes) {
        // Per-object material parameters
        glUniform3fv(g_scene_uniforms.material_ambient, 1, mesh.ambient.data());
        glUniform3fv(g_scene_uniforms.material_diffuse, 1, mesh.diffuse.data());
        glUniform3fv(g_scene_uniforms.material_specular, 1, mesh.specular.data());
        glUniform1f (g_scene_uniforms.material_shininess, mesh.shininess);

        // Draw
        glBindVertexArray(mesh.vao);
        glDrawArrays(GL_TRIANGLES, 0, mesh.vertex_count);
    }
    glBindVertexArray(0);
}

void render_normal_map_mode() {
    // Render a PNG with a normal map
    glUseProgram(g_quad_program);

    // Model-view from arcball rotation and a simple camera translate moving back
    const Eigen::Matrix4d arcball_data = g_arcball.rotation().to_matrix();
    Eigen::Matrix4f model = to_matrix4f(arcball_data);
    Eigen::Matrix4f view  = make_translation(0.0f, 0.0f, -3.0f);
    Eigen::Matrix4f model_view = view * model;

    // Perspective from window aspect
    float aspect = std::max(1, g_window_width) / static_cast<float>(std::max(1, g_window_height));
    Eigen::Matrix4f projection = make_perspective(45.0f, aspect, 0.1f, 20.0f);

    // Normal matrix for this model view
    Eigen::Matrix3f normal_matrix = model_view.block<3,3>(0,0).inverse().transpose();

    // Upload transforms
    glUniformMatrix4fv(g_quad_uniforms.model_view, 1, GL_FALSE, model_view.data());
    glUniformMatrix4fv(g_quad_uniforms.projection, 1, GL_FALSE, projection.data());
    glUniformMatrix3fv(g_quad_uniforms.normal_matrix,1, GL_FALSE, normal_matrix.data());

    // Simple lighting & material constants for the quad shader
    glUniform3f (g_quad_uniforms.light_position, 0.0f, 0.0f, 3.0f);
    glUniform3f (g_quad_uniforms.light_color, 1.0f, 1.0f, 1.0f);
    glUniform3fv(g_quad_uniforms.ambient_light, 1, g_ambient_light.data());
    glUniform3f (g_quad_uniforms.specular, 0.4f, 0.4f, 0.4f);
    glUniform1f (g_quad_uniforms.shininess, 32.0f);

    // Bind textures: color to unit 0, normal map to unit 1
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g_quad_state.color_tex);
    glUniform1i(g_quad_uniforms.color_texture, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, g_quad_state.normal_tex);
    glUniform1i(g_quad_uniforms.normal_texture, 1);

    // Draw the indexed quad
    glBindVertexArray(g_quad_state.vao);
    glDrawElements(GL_TRIANGLES, g_quad_state.index_count, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

// #####################
//  GLUT Helpers
// #####################

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (g_mode == RunMode::Scene) {
        render_scene_mode();
    } else {
        render_normal_map_mode();
    }
    glutSwapBuffers();
}

double camera_aspect() {
    const Eigen::Matrix4d& P = g_scene_state.scene.cam_transforms.P;
    return P(1,1) / P(0,0);
}

// I found this code, but it helps to make my renderings look
// properly proportioned on the screen at any window size.
// It essentially calculates if it needs to pad the window at all in order
// to match the rendering to the necessary aspect ratio, instead of warping things
void apply_scene_viewport(int width, int height) {
    double target = camera_aspect();
    double win_aspect = static_cast<double>(width) / static_cast<double>(height);

    int vx = 0, vy = 0, vw = width, vh = height;
    if (win_aspect > target) {
        // window is wider -> pillarbox left/right
        vw = static_cast<int>(std::round(vh * target));
        vx = (width - vw) / 2;
    } else if (win_aspect < target) {
        // window is taller -> letterbox top/bottom
        vh = static_cast<int>(std::round(vw / target));
        vy = (height - vh) / 2;
    }
    glViewport(vx, vy, vw, vh);
    g_arcball.set_viewport(vx, vy, vw, vh);
}

void reshape(int width, int height) {
    g_window_width = std::max(width, 1);
    g_window_height = std::max(height, 1);
    if (g_mode == RunMode::Scene) {
        apply_scene_viewport(g_window_width, g_window_height);
    } else {
        // No special aspect ratio considerations needed
        glViewport(0, 0, g_window_width, g_window_height);
        g_arcball.set_viewport(0, 0, g_window_width, g_window_height);
    }
    g_arcball.set_window(g_window_width, g_window_height);
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            g_arcball.begin_drag(x, y);
        } else if (state == GLUT_UP) {
            g_arcball.end_drag();
        }
        glutPostRedisplay();
    }
}

void motion(int x, int y) {
    g_arcball.update_drag(x, y);
    glutPostRedisplay();
}

void keyboard(unsigned char key, int, int) {
    if (key == 27 || key == 'q' || key == 'Q') {
        std::exit(0);
    }
}

// #####################
//  Main Program
// #####################

void init_common_gl_state() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    // glEnable(GL_BACK);
}

void setup_scene_mode() {
    build_scene_meshes();
    init_scene_lights();
    create_scene_program();
    init_common_gl_state();
    glClearColor(0.f, 0.f, 0.f, 1.f);
    apply_scene_viewport(g_window_width, g_window_height);
}

void setup_normal_map_mode() {
    build_quad_geometry();
    create_quad_program();
    g_quad_state.color_tex = load_png_texture(g_color_path);
    g_quad_state.normal_tex = load_png_texture(g_normal_path);
    init_common_gl_state();
    glClearColor(0.2f, 0.f, 0.f, 1.f);
    glViewport(0, 0, g_window_width, g_window_height);
    g_arcball.set_viewport(0, 0, g_window_width, g_window_height);
}

std::size_t parse_size(const char* text) {
    long value = std::strtol(text, nullptr, 10);
    if (value <= 0) {
        throw std::runtime_error("Resolution must be positive");
    }
    return static_cast<std::size_t>(value);
}

int main(int argc, char** argv) {
    try {
        if (argc == 5) {
            g_mode = RunMode::Scene;
            g_scene_path = argv[1];
            std::ifstream fin(g_scene_path);
            if (!fin) {
                std::cerr << "Could not open scene file: " << g_scene_path << "\n";
                return 1;
            }
            g_scene_state.scene = parse_scene_file(fin, parse_parent_path(g_scene_path));
            g_window_width = static_cast<int>(parse_size(argv[2]));
            g_window_height = static_cast<int>(parse_size(argv[3]));
            g_shading_mode = std::atoi(argv[4]) == 0 ? 0 : 1;
            g_arcball.set_window(g_window_width, g_window_height);
        } else if (argc == 3) {
            g_mode = RunMode::NormalMap;
            g_color_path = argv[1];
            g_normal_path = argv[2];
            g_window_width = 800;
            g_window_height = 600;
            g_arcball.set_window(g_window_width, g_window_height);
        } else {
            std::cerr << "Usage: " << argv[0] << " [scene.txt] [xres] [yres] [mode]\n"
                        << " or: " << argv[0] << " [color.png] [normal.png]\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    if (const char* override_dir = std::getenv("HW4_SHADER_DIR")) {
        if (*override_dir != '\0') {
            g_shader_dir = override_dir;
        }
    } // Defaults to SHADER_DIR

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_window_width, g_window_height);
    glutCreateWindow("HW4 Renderer");

    #ifdef __APPLE__
        // Fixes mac segfaults
        glewExperimental = GL_TRUE;
    #endif

    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "GLEW init error: " << glewGetErrorString(err) << "\n";
        return 1;
    }

    glGetError();

    if (g_mode == RunMode::Scene) {
        setup_scene_mode();
    } else {
        setup_normal_map_mode();
    }

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
