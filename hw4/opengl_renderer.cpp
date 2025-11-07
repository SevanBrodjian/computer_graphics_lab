#include "arcball.h"
#include "scene_loader.h"
#include "texture_loader.h"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
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
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kMaxLights = 8;
constexpr float kPi = 3.14159265358979323846f;

enum class ProgramMode {
    Scene,
    NormalMapped
};

struct DrawableObject {
    GLuint vbo_positions{0};
    GLuint vbo_normals{0};
    GLsizei vertex_count{0};
    Eigen::Vector3f ambient{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f diffuse{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f specular{0.0f, 0.0f, 0.0f};
    float shininess{1.0f};
};

struct SceneRenderer {
    Scene scene;
    std::vector<DrawableObject> drawables;
    Arcball arcball;
    ProgramMode mode{ProgramMode::Scene};
    int shading_mode{0};

    int window_width{800};
    int window_height{800};

    GLuint shader_program{0};
    GLint attr_position{-1};
    GLint attr_normal{-1};

    GLint uniform_model_view{-1};
    GLint uniform_projection{-1};
    GLint uniform_normal_matrix{-1};
    GLint uniform_light_count{-1};
    GLint uniform_light_positions{-1};
    GLint uniform_light_colors{-1};
    GLint uniform_light_atten{-1};
    GLint uniform_material_ambient{-1};
    GLint uniform_material_diffuse{-1};
    GLint uniform_material_specular{-1};
    GLint uniform_material_shininess{-1};
    GLint uniform_ambient_light{-1};
    GLint uniform_shading_mode{-1};
};

struct QuadRenderer {
    Arcball arcball;
    int window_width{800};
    int window_height{800};

    GLuint shader_program{0};
    GLuint vbo{0};
    GLuint ibo{0};
    GLuint color_texture{0};
    GLuint normal_texture{0};

    GLint attr_position{-1};
    GLint attr_normal{-1};
    GLint attr_tangent{-1};
    GLint attr_bitangent{-1};
    GLint attr_texcoord{-1};

    GLint uniform_model_view{-1};
    GLint uniform_projection{-1};
    GLint uniform_normal_matrix{-1};
    GLint uniform_color_tex{-1};
    GLint uniform_normal_tex{-1};
    GLint uniform_light_pos{-1};
    GLint uniform_light_color{-1};
    GLint uniform_ambient_light{-1};
    GLint uniform_material_specular{-1};
    GLint uniform_material_shininess{-1};

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
};

ProgramMode g_program_mode = ProgramMode::Scene;
SceneRenderer g_scene_renderer;
QuadRenderer g_quad_renderer;
std::string g_color_texture_path;
std::string g_normal_texture_path;

// Utility helpers -----------------------------------------------------------

GLuint compile_shader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success != GL_TRUE) {
        GLint log_length = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
        std::vector<char> log(log_length + 1, '\0');
        glGetShaderInfoLog(shader, log_length, nullptr, log.data());
        std::string message = (type == GL_VERTEX_SHADER ? "Vertex" : "Fragment");
        message += " shader compilation failed:\n";
        message += log.data();
        throw std::runtime_error(message);
    }
    return shader;
}

GLuint link_program(GLuint vertex_shader, GLuint fragment_shader,
                    const std::vector<std::pair<GLuint, const char*>>& attributes = {}) {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);

    for (const auto& attr : attributes) {
        glBindAttribLocation(program, attr.first, attr.second);
    }

    glLinkProgram(program);
    GLint success = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (success != GL_TRUE) {
        GLint log_length = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
        std::vector<char> log(log_length + 1, '\0');
        glGetProgramInfoLog(program, log_length, nullptr, log.data());
        std::string message = "Program link failed:\n";
        message += log.data();
        throw std::runtime_error(message);
    }
    return program;
}

Eigen::Matrix4f to_matrix4f(const Eigen::Matrix4d& m) {
    return m.cast<float>();
}

Eigen::Matrix3f normal_matrix_from(const Eigen::Matrix4f& model_view) {
    Eigen::Matrix3f upper = model_view.block<3,3>(0,0);
    return upper.inverse().transpose();
}

Eigen::Matrix4f matrix_from_arcball(const Arcball& arcball) {
    auto arr = arcball.rotation().to_matrix();
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            mat(row, col) = static_cast<float>(arr[col * 4 + row]);
        }
    }
    return mat;
}

Eigen::Matrix4f perspective(float fovy_radians, float aspect, float znear, float zfar) {
    float f = 1.0f / std::tan(fovy_radians * 0.5f);
    Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
    m(0, 0) = f / aspect;
    m(1, 1) = f;
    m(2, 2) = (zfar + znear) / (znear - zfar);
    m(2, 3) = (2.0f * zfar * znear) / (znear - zfar);
    m(3, 2) = -1.0f;
    return m;
}

Eigen::Matrix4f look_at(const Eigen::Vector3f& eye,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& up) {
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<1,3>(0,0) = s.transpose();
    m.block<1,3>(1,0) = u.transpose();
    m.block<1,3>(2,0) = (-f).transpose();

    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0, 3) = -eye.x();
    translate(1, 3) = -eye.y();
    translate(2, 3) = -eye.z();

    return m * translate;
}

float camera_aspect_from_P(const Eigen::Matrix4d& P) {
    return static_cast<float>(P(1,1) / P(0,0));
}

struct Viewport {
    int x{0};
    int y{0};
    int width{1};
    int height{1};
};

Viewport apply_letterboxed_viewport(SceneRenderer& renderer, int win_w, int win_h) {
    const float cam_aspect = camera_aspect_from_P(renderer.scene.cam_transforms.P);
    const float window_aspect = static_cast<float>(win_w) / std::max(win_h, 1);

    Viewport vp;
    vp.width = win_w;
    vp.height = win_h;
    if (window_aspect > cam_aspect) {
        vp.height = win_h;
        vp.width = static_cast<int>(std::round(vp.height * cam_aspect));
        vp.x = (win_w - vp.width) / 2;
        vp.y = 0;
    } else if (window_aspect < cam_aspect) {
        vp.width = win_w;
        vp.height = static_cast<int>(std::round(vp.width / cam_aspect));
        vp.x = 0;
        vp.y = (win_h - vp.height) / 2;
    }
    glViewport(vp.x, vp.y, std::max(vp.width,1), std::max(vp.height,1));
    renderer.arcball.set_viewport(vp.x, vp.y, std::max(vp.width,1), std::max(vp.height,1));
    return vp;
}

// Scene renderer ------------------------------------------------------------

const char* kSceneVertexShader = R"GLSL(
#version 120

attribute vec3 aPosition;
attribute vec3 aNormal;

uniform mat4 uModelView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;
uniform int uShadingMode;

const int MAX_LIGHTS = 8;
uniform int uLightCount;
uniform vec3 uLightPosition[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];
uniform float uLightAtten[MAX_LIGHTS];

uniform vec3 uMaterialAmbient;
uniform vec3 uMaterialDiffuse;
uniform vec3 uMaterialSpecular;
uniform float uMaterialShininess;
uniform vec3 uAmbientLight;

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vVertexColor;

vec3 applyLighting(vec3 position, vec3 normal) {
    vec3 viewDir = normalize(-position);
    vec3 color = uMaterialAmbient * uAmbientLight;
    for (int i = 0; i < MAX_LIGHTS; ++i) {
        if (i >= uLightCount) break;
        vec3 lightVec = uLightPosition[i] - position;
        float dist = length(lightVec);
        if (dist > 0.0) {
            lightVec /= dist;
        }
        float atten = 1.0 / (1.0 + uLightAtten[i] * dist * dist);
        float diff = max(dot(normal, lightVec), 0.0);
        vec3 reflectDir = reflect(-lightVec, normal);
        float spec = 0.0;
        if (diff > 0.0) {
            spec = pow(max(dot(viewDir, reflectDir), 0.0), max(uMaterialShininess, 0.0));
        }
        vec3 lightColor = uLightColor[i];
        color += atten * lightColor * (uMaterialDiffuse * diff + uMaterialSpecular * spec);
    }
    return clamp(color, 0.0, 1.0);
}

void main() {
    vec4 viewPos = uModelView * vec4(aPosition, 1.0);
    vec3 normal = normalize(uNormalMatrix * aNormal);
    vPosition = viewPos.xyz;
    vNormal = normal;
    if (uShadingMode == 0) {
        vVertexColor = applyLighting(viewPos.xyz, normal);
    } else {
        vVertexColor = vec3(0.0);
    }
    gl_Position = uProjection * viewPos;
}
)GLSL";

const char* kSceneFragmentShader = R"GLSL(
#version 120

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vVertexColor;

const int MAX_LIGHTS = 8;
uniform int uLightCount;
uniform vec3 uLightPosition[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];
uniform float uLightAtten[MAX_LIGHTS];

uniform vec3 uMaterialAmbient;
uniform vec3 uMaterialDiffuse;
uniform vec3 uMaterialSpecular;
uniform float uMaterialShininess;
uniform vec3 uAmbientLight;
uniform int uShadingMode;

vec3 applyLighting(vec3 position, vec3 normal) {
    vec3 viewDir = normalize(-position);
    vec3 color = uMaterialAmbient * uAmbientLight;
    for (int i = 0; i < MAX_LIGHTS; ++i) {
        if (i >= uLightCount) break;
        vec3 lightVec = uLightPosition[i] - position;
        float dist = length(lightVec);
        if (dist > 0.0) {
            lightVec /= dist;
        }
        float atten = 1.0 / (1.0 + uLightAtten[i] * dist * dist);
        float diff = max(dot(normal, lightVec), 0.0);
        vec3 reflectDir = reflect(-lightVec, normal);
        float spec = 0.0;
        if (diff > 0.0) {
            spec = pow(max(dot(viewDir, reflectDir), 0.0), max(uMaterialShininess, 0.0));
        }
        vec3 lightColor = uLightColor[i];
        color += atten * lightColor * (uMaterialDiffuse * diff + uMaterialSpecular * spec);
    }
    return clamp(color, 0.0, 1.0);
}

void main() {
    vec3 normal = normalize(vNormal);
    vec3 color = vVertexColor;
    if (uShadingMode != 0) {
        color = applyLighting(vPosition, normal);
    }
    gl_FragColor = vec4(color, 1.0);
}
)GLSL";

void destroy_scene_drawables(SceneRenderer& renderer) {
    for (auto& drawable : renderer.drawables) {
        if (drawable.vbo_positions) {
            glDeleteBuffers(1, &drawable.vbo_positions);
            drawable.vbo_positions = 0;
        }
        if (drawable.vbo_normals) {
            glDeleteBuffers(1, &drawable.vbo_normals);
            drawable.vbo_normals = 0;
        }
    }
    renderer.drawables.clear();
}

void build_scene_drawables(SceneRenderer& renderer) {
    destroy_scene_drawables(renderer);
    renderer.drawables.reserve(renderer.scene.scene_objects.size());

    for (const auto& inst : renderer.scene.scene_objects) {
        DrawableObject drawable;
        std::vector<GLfloat> positions;
        std::vector<GLfloat> normals;
        positions.reserve(inst.obj.faces.size() * 9);
        normals.reserve(inst.obj.faces.size() * 9);

        auto fill_material = [](const Eigen::Vector3d& src) {
            return Eigen::Vector3f(static_cast<float>(src[0]),
                                   static_cast<float>(src[1]),
                                   static_cast<float>(src[2]));
        };
        drawable.ambient = fill_material(inst.ambient);
        drawable.diffuse = fill_material(inst.diffuse);
        drawable.specular = fill_material(inst.specular);
        drawable.shininess = static_cast<float>(std::clamp(inst.shininess, 0.0, 128.0));

        for (const auto& face : inst.obj.faces) {
            const Vertex& v1 = inst.obj.vertices[face.v1];
            const Vertex& v2 = inst.obj.vertices[face.v2];
            const Vertex& v3 = inst.obj.vertices[face.v3];
            const Normal& n1 = inst.obj.normals[face.vn1];
            const Normal& n2 = inst.obj.normals[face.vn2];
            const Normal& n3 = inst.obj.normals[face.vn3];

            const std::array<const Vertex*, 3> verts{&v1, &v2, &v3};
            const std::array<const Normal*, 3> norms{&n1, &n2, &n3};
            for (int i = 0; i < 3; ++i) {
                positions.push_back(static_cast<GLfloat>(verts[i]->x));
                positions.push_back(static_cast<GLfloat>(verts[i]->y));
                positions.push_back(static_cast<GLfloat>(verts[i]->z));

                normals.push_back(static_cast<GLfloat>(norms[i]->x));
                normals.push_back(static_cast<GLfloat>(norms[i]->y));
                normals.push_back(static_cast<GLfloat>(norms[i]->z));
            }
        }

        drawable.vertex_count = static_cast<GLsizei>(positions.size() / 3);
        glGenBuffers(1, &drawable.vbo_positions);
        glBindBuffer(GL_ARRAY_BUFFER, drawable.vbo_positions);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(GLfloat), positions.data(), GL_STATIC_DRAW);

        glGenBuffers(1, &drawable.vbo_normals);
        glBindBuffer(GL_ARRAY_BUFFER, drawable.vbo_normals);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLfloat), normals.data(), GL_STATIC_DRAW);

        renderer.drawables.push_back(std::move(drawable));
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void init_scene_shader(SceneRenderer& renderer) {
    GLuint vs = compile_shader(GL_VERTEX_SHADER, kSceneVertexShader);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, kSceneFragmentShader);

    renderer.shader_program = link_program(vs, fs, {
        {0, "aPosition"},
        {1, "aNormal"}
    });

    glDeleteShader(vs);
    glDeleteShader(fs);

    renderer.attr_position = 0;
    renderer.attr_normal = 1;

    renderer.uniform_model_view = glGetUniformLocation(renderer.shader_program, "uModelView");
    renderer.uniform_projection = glGetUniformLocation(renderer.shader_program, "uProjection");
    renderer.uniform_normal_matrix = glGetUniformLocation(renderer.shader_program, "uNormalMatrix");
    renderer.uniform_light_count = glGetUniformLocation(renderer.shader_program, "uLightCount");
    renderer.uniform_light_positions = glGetUniformLocation(renderer.shader_program, "uLightPosition");
    renderer.uniform_light_colors = glGetUniformLocation(renderer.shader_program, "uLightColor");
    renderer.uniform_light_atten = glGetUniformLocation(renderer.shader_program, "uLightAtten");
    renderer.uniform_material_ambient = glGetUniformLocation(renderer.shader_program, "uMaterialAmbient");
    renderer.uniform_material_diffuse = glGetUniformLocation(renderer.shader_program, "uMaterialDiffuse");
    renderer.uniform_material_specular = glGetUniformLocation(renderer.shader_program, "uMaterialSpecular");
    renderer.uniform_material_shininess = glGetUniformLocation(renderer.shader_program, "uMaterialShininess");
    renderer.uniform_ambient_light = glGetUniformLocation(renderer.shader_program, "uAmbientLight");
    renderer.uniform_shading_mode = glGetUniformLocation(renderer.shader_program, "uShadingMode");
}

void init_scene_renderer(SceneRenderer& renderer) {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    init_scene_shader(renderer);
    build_scene_drawables(renderer);
}

void render_scene(SceneRenderer& renderer) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(renderer.shader_program);

    Eigen::Matrix4f view = to_matrix4f(renderer.scene.cam_transforms.Cinv);
    Eigen::Matrix4f projection = to_matrix4f(renderer.scene.cam_transforms.P);
    Eigen::Matrix4f model = matrix_from_arcball(renderer.arcball);
    Eigen::Matrix4f model_view = view * model;
    Eigen::Matrix3f normal_matrix = normal_matrix_from(model_view);

    glUniformMatrix4fv(renderer.uniform_model_view, 1, GL_FALSE, model_view.data());
    glUniformMatrix4fv(renderer.uniform_projection, 1, GL_FALSE, projection.data());
    glUniformMatrix3fv(renderer.uniform_normal_matrix, 1, GL_FALSE, normal_matrix.data());
    glUniform1i(renderer.uniform_shading_mode, renderer.shading_mode);

    const int light_count = std::min<int>(renderer.scene.lights.size(), kMaxLights);
    std::vector<Eigen::Vector3f> light_positions(light_count);
    std::vector<Eigen::Vector3f> light_colors(light_count);
    std::vector<float> light_atten(light_count);

    for (int i = 0; i < light_count; ++i) {
        const auto& light = renderer.scene.lights[i];
        Eigen::Vector4f pos(static_cast<float>(light.x),
                            static_cast<float>(light.y),
                            static_cast<float>(light.z),
                            1.0f);
        pos = model * pos; // rotate with the arcball
        pos = view * pos;
        light_positions[i] = pos.head<3>();
        light_colors[i] = Eigen::Vector3f(static_cast<float>(light.r),
                                          static_cast<float>(light.g),
                                          static_cast<float>(light.b));
        light_atten[i] = static_cast<float>(light.atten);
    }

    glUniform1i(renderer.uniform_light_count, light_count);
    if (light_count > 0) {
        glUniform3fv(renderer.uniform_light_positions, light_count, light_positions[0].data());
        glUniform3fv(renderer.uniform_light_colors, light_count, light_colors[0].data());
        glUniform1fv(renderer.uniform_light_atten, light_count, light_atten.data());
    }

    Eigen::Vector3f ambient_light(1.0f, 1.0f, 1.0f);
    glUniform3fv(renderer.uniform_ambient_light, 1, ambient_light.data());

    for (const auto& drawable : renderer.drawables) {
        glUniform3fv(renderer.uniform_material_ambient, 1, drawable.ambient.data());
        glUniform3fv(renderer.uniform_material_diffuse, 1, drawable.diffuse.data());
        glUniform3fv(renderer.uniform_material_specular, 1, drawable.specular.data());
        glUniform1f(renderer.uniform_material_shininess, drawable.shininess);

        glBindBuffer(GL_ARRAY_BUFFER, drawable.vbo_positions);
        glEnableVertexAttribArray(renderer.attr_position);
        glVertexAttribPointer(renderer.attr_position, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, drawable.vbo_normals);
        glEnableVertexAttribArray(renderer.attr_normal);
        glVertexAttribPointer(renderer.attr_normal, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glDrawArrays(GL_TRIANGLES, 0, drawable.vertex_count);

        glDisableVertexAttribArray(renderer.attr_position);
        glDisableVertexAttribArray(renderer.attr_normal);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);

    glutSwapBuffers();
}

// Normal mapped quad -------------------------------------------------------

const char* kQuadVertexShader = R"GLSL(
#version 120

attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTangent;
attribute vec3 aBitangent;
attribute vec2 aTexCoord;

uniform mat4 uModelView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vTangent;
varying vec3 vBitangent;
varying vec2 vTexCoord;

void main() {
    vec4 viewPos = uModelView * vec4(aPosition, 1.0);
    vPosition = viewPos.xyz;
    vNormal = normalize(uNormalMatrix * aNormal);
    vTangent = normalize(uNormalMatrix * aTangent);
    vBitangent = normalize(uNormalMatrix * aBitangent);
    vTexCoord = aTexCoord;
    gl_Position = uProjection * viewPos;
}
)GLSL";

const char* kQuadFragmentShader = R"GLSL(
#version 120

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vTangent;
varying vec3 vBitangent;
varying vec2 vTexCoord;

uniform sampler2D uColorTex;
uniform sampler2D uNormalTex;
uniform vec3 uLightPos;
uniform vec3 uLightColor;
uniform vec3 uAmbientLight;
uniform vec3 uMaterialSpecular;
uniform float uMaterialShininess;

vec3 applyLighting(vec3 position, vec3 normal, vec3 baseColor) {
    vec3 viewDir = normalize(-position);
    vec3 lightVec = uLightPos - position;
    float dist = length(lightVec);
    if (dist > 0.0) {
        lightVec /= dist;
    }
    float diff = max(dot(normal, lightVec), 0.0);
    vec3 reflectDir = reflect(-lightVec, normal);
    float spec = 0.0;
    if (diff > 0.0) {
        spec = pow(max(dot(viewDir, reflectDir), 0.0), max(uMaterialShininess, 0.0));
    }

    vec3 color = baseColor * uAmbientLight;
    color += baseColor * diff * uLightColor;
    color += uMaterialSpecular * spec * uLightColor;
    return clamp(color, 0.0, 1.0);
}

void main() {
    vec3 tangent = normalize(vTangent);
    vec3 bitangent = normalize(vBitangent);
    vec3 normal = normalize(vNormal);
    mat3 TBN = mat3(tangent, bitangent, normal);

    vec3 sampledNormal = texture2D(uNormalTex, vTexCoord).rgb;
    sampledNormal = normalize(sampledNormal * 2.0 - 1.0);
    vec3 worldNormal = normalize(TBN * sampledNormal);

    vec3 baseColor = texture2D(uColorTex, vTexCoord).rgb;
    vec3 color = applyLighting(vPosition, worldNormal, baseColor);
    gl_FragColor = vec4(color, 1.0);
}
)GLSL";

void destroy_quad_buffers(QuadRenderer& renderer) {
    if (renderer.vbo) {
        glDeleteBuffers(1, &renderer.vbo);
        renderer.vbo = 0;
    }
    if (renderer.ibo) {
        glDeleteBuffers(1, &renderer.ibo);
        renderer.ibo = 0;
    }
    if (renderer.color_texture) {
        glDeleteTextures(1, &renderer.color_texture);
        renderer.color_texture = 0;
    }
    if (renderer.normal_texture) {
        glDeleteTextures(1, &renderer.normal_texture);
        renderer.normal_texture = 0;
    }
}

void init_quad_renderer(QuadRenderer& renderer) {
    GLuint vs = compile_shader(GL_VERTEX_SHADER, kQuadVertexShader);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, kQuadFragmentShader);
    renderer.shader_program = link_program(vs, fs, {
        {0, "aPosition"},
        {1, "aNormal"},
        {2, "aTangent"},
        {3, "aBitangent"},
        {4, "aTexCoord"}
    });
    glDeleteShader(vs);
    glDeleteShader(fs);

    renderer.attr_position = 0;
    renderer.attr_normal = 1;
    renderer.attr_tangent = 2;
    renderer.attr_bitangent = 3;
    renderer.attr_texcoord = 4;

    renderer.uniform_model_view = glGetUniformLocation(renderer.shader_program, "uModelView");
    renderer.uniform_projection = glGetUniformLocation(renderer.shader_program, "uProjection");
    renderer.uniform_normal_matrix = glGetUniformLocation(renderer.shader_program, "uNormalMatrix");
    renderer.uniform_color_tex = glGetUniformLocation(renderer.shader_program, "uColorTex");
    renderer.uniform_normal_tex = glGetUniformLocation(renderer.shader_program, "uNormalTex");
    renderer.uniform_light_pos = glGetUniformLocation(renderer.shader_program, "uLightPos");
    renderer.uniform_light_color = glGetUniformLocation(renderer.shader_program, "uLightColor");
    renderer.uniform_ambient_light = glGetUniformLocation(renderer.shader_program, "uAmbientLight");
    renderer.uniform_material_specular = glGetUniformLocation(renderer.shader_program, "uMaterialSpecular");
    renderer.uniform_material_shininess = glGetUniformLocation(renderer.shader_program, "uMaterialShininess");

    struct Vertex {
        GLfloat position[3];
        GLfloat normal[3];
        GLfloat tangent[3];
        GLfloat bitangent[3];
        GLfloat texcoord[2];
    };

    const std::array<Vertex, 4> vertices = {{{
        {-1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}
    },{
        {1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}
    },{
        {1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}
    },{
        {-1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}
    }}};

    const std::array<GLushort, 6> indices = {0, 1, 2, 0, 2, 3};

    glGenBuffers(1, &renderer.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, renderer.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &renderer.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderer.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    renderer.color_texture = load_png_texture(g_color_texture_path);
    renderer.normal_texture = load_png_texture(g_normal_texture_path);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

    renderer.view = look_at({0.0f, 0.0f, 4.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    renderer.projection = perspective(kPi / 4.0f, 1.0f, 0.1f, 100.0f);
}

void render_quad(QuadRenderer& renderer) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(renderer.shader_program);

    Eigen::Matrix4f model = matrix_from_arcball(renderer.arcball);
    Eigen::Matrix4f model_view = renderer.view * model;
    Eigen::Matrix3f normal_matrix = normal_matrix_from(model_view);

    glUniformMatrix4fv(renderer.uniform_model_view, 1, GL_FALSE, model_view.data());
    glUniformMatrix4fv(renderer.uniform_projection, 1, GL_FALSE, renderer.projection.data());
    glUniformMatrix3fv(renderer.uniform_normal_matrix, 1, GL_FALSE, normal_matrix.data());

    Eigen::Vector3f light_pos = (renderer.view * Eigen::Vector4f(0.0f, 0.0f, 4.0f, 1.0f)).head<3>();
    Eigen::Vector3f light_color(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f ambient(0.2f, 0.2f, 0.2f);
    Eigen::Vector3f specular(0.4f, 0.4f, 0.4f);
    float shininess = 32.0f;

    glUniform3fv(renderer.uniform_light_pos, 1, light_pos.data());
    glUniform3fv(renderer.uniform_light_color, 1, light_color.data());
    glUniform3fv(renderer.uniform_ambient_light, 1, ambient.data());
    glUniform3fv(renderer.uniform_material_specular, 1, specular.data());
    glUniform1f(renderer.uniform_material_shininess, shininess);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, renderer.color_texture);
    glUniform1i(renderer.uniform_color_tex, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, renderer.normal_texture);
    glUniform1i(renderer.uniform_normal_tex, 1);

    glBindBuffer(GL_ARRAY_BUFFER, renderer.vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderer.ibo);

    const GLsizei stride = sizeof(float) * (3 + 3 + 3 + 3 + 2);
    std::size_t offset = 0;
    glEnableVertexAttribArray(renderer.attr_position);
    glVertexAttribPointer(renderer.attr_position, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));

    offset += sizeof(float) * 3;
    glEnableVertexAttribArray(renderer.attr_normal);
    glVertexAttribPointer(renderer.attr_normal, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));

    offset += sizeof(float) * 3;
    glEnableVertexAttribArray(renderer.attr_tangent);
    glVertexAttribPointer(renderer.attr_tangent, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));

    offset += sizeof(float) * 3;
    glEnableVertexAttribArray(renderer.attr_bitangent);
    glVertexAttribPointer(renderer.attr_bitangent, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));

    offset += sizeof(float) * 3;
    glEnableVertexAttribArray(renderer.attr_texcoord);
    glVertexAttribPointer(renderer.attr_texcoord, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);

    glDisableVertexAttribArray(renderer.attr_position);
    glDisableVertexAttribArray(renderer.attr_normal);
    glDisableVertexAttribArray(renderer.attr_tangent);
    glDisableVertexAttribArray(renderer.attr_bitangent);
    glDisableVertexAttribArray(renderer.attr_texcoord);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);

    glutSwapBuffers();
}

// GLUT callbacks ------------------------------------------------------------

void display_callback() {
    if (g_program_mode == ProgramMode::Scene) {
        render_scene(g_scene_renderer);
    } else {
        render_quad(g_quad_renderer);
    }
}

void reshape_callback(int width, int height) {
    if (width <= 0) width = 1;
    if (height <= 0) height = 1;

    if (g_program_mode == ProgramMode::Scene) {
        g_scene_renderer.window_width = width;
        g_scene_renderer.window_height = height;
        g_scene_renderer.arcball.set_window(width, height);
        auto vp = apply_letterboxed_viewport(g_scene_renderer, width, height);
        (void)vp;
    } else {
        g_quad_renderer.window_width = width;
        g_quad_renderer.window_height = height;
        g_quad_renderer.arcball.set_window(width, height);
        g_quad_renderer.arcball.set_viewport(0, 0, width, height);
        glViewport(0, 0, width, height);
        float aspect = static_cast<float>(width) / static_cast<float>(height);
        g_quad_renderer.projection = perspective(kPi / 4.0f, aspect, 0.1f, 100.0f);
    }
    glutPostRedisplay();
}

void mouse_callback(int button, int state, int x, int y) {
    if (button != GLUT_LEFT_BUTTON) return;
    if (state == GLUT_DOWN) {
        if (g_program_mode == ProgramMode::Scene) {
            g_scene_renderer.arcball.begin_drag(x, y);
        } else {
            g_quad_renderer.arcball.begin_drag(x, y);
        }
    } else if (state == GLUT_UP) {
        if (g_program_mode == ProgramMode::Scene) {
            g_scene_renderer.arcball.end_drag();
        } else {
            g_quad_renderer.arcball.end_drag();
        }
    }
    glutPostRedisplay();
}

void motion_callback(int x, int y) {
    if (g_program_mode == ProgramMode::Scene) {
        g_scene_renderer.arcball.update_drag(x, y);
    } else {
        g_quad_renderer.arcball.update_drag(x, y);
    }
    glutPostRedisplay();
}

void keyboard_callback(unsigned char key, int, int) {
    if (key == 27 || key == 'q' || key == 'Q') {
        destroy_scene_drawables(g_scene_renderer);
        destroy_quad_buffers(g_quad_renderer);
        std::exit(0);
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 5) {
        g_program_mode = ProgramMode::Scene;
    } else if (argc == 3) {
        g_program_mode = ProgramMode::NormalMapped;
    } else {
        std::cerr << "Usage (scene): " << argv[0] << " [scene.txt] [xres] [yres] [mode]\n";
        std::cerr << "Usage (normal map): " << argv[0] << " [color_texture.png] [normal_map.png]\n";
        return 1;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    if (g_program_mode == ProgramMode::Scene) {
        const std::size_t xres = parse_size_t(argv[2]);
        const std::size_t yres = parse_size_t(argv[3]);
        int mode = 0;
        try {
            mode = std::stoi(argv[4]);
        } catch (const std::exception&) {
            std::cerr << "Mode must be an integer (0 for Gouraud, 1 for Phong)\n";
            return 1;
        }
        if (mode != 0 && mode != 1) {
            std::cerr << "Mode must be 0 (Gouraud) or 1 (Phong)\n";
            return 1;
        }

        std::ifstream fin(argv[1]);
        if (!fin) {
            std::cerr << "Could not open scene file: " << argv[1] << "\n";
            return 1;
        }

        try {
            g_scene_renderer.scene = parse_scene_file(fin, parse_parent_path(argv[1]));
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse scene: " << e.what() << "\n";
            return 1;
        }

        g_scene_renderer.shading_mode = mode;
        g_scene_renderer.window_width = static_cast<int>(xres);
        g_scene_renderer.window_height = static_cast<int>(yres);
        g_scene_renderer.arcball.set_window(g_scene_renderer.window_width, g_scene_renderer.window_height);

        glutInitWindowSize(g_scene_renderer.window_width, g_scene_renderer.window_height);
        glutInitWindowPosition(0, 0);
        glutCreateWindow("OpenGL Scene Renderer");
    } else {
        g_color_texture_path = argv[1];
        g_normal_texture_path = argv[2];

        g_quad_renderer.window_width = 800;
        g_quad_renderer.window_height = 800;
        g_quad_renderer.arcball.set_window(g_quad_renderer.window_width, g_quad_renderer.window_height);

        glutInitWindowSize(g_quad_renderer.window_width, g_quad_renderer.window_height);
        glutInitWindowPosition(0, 0);
        glutCreateWindow("Normal Mapped Quad");
    }

    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "GLEW init error: " << glewGetErrorString(err) << "\n";
        return 1;
    }

    try {
        if (g_program_mode == ProgramMode::Scene) {
            init_scene_renderer(g_scene_renderer);
            apply_letterboxed_viewport(g_scene_renderer, g_scene_renderer.window_width, g_scene_renderer.window_height);
        } else {
            init_quad_renderer(g_quad_renderer);
            reshape_callback(g_quad_renderer.window_width, g_quad_renderer.window_height);
        }
    } catch (const std::exception& e) {
        std::cerr << "Initialization error: " << e.what() << "\n";
        return 1;
    }

    glutDisplayFunc(display_callback);
    glutReshapeFunc(reshape_callback);
    glutMouseFunc(mouse_callback);
    glutMotionFunc(motion_callback);
    glutKeyboardFunc(keyboard_callback);

    glutMainLoop();
    return 0;
}
