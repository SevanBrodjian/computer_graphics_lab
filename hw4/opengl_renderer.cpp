#include "scene_loader.h"
#include "arcball.h"
#include "shader_program.h"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <Eigen/Dense>

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

struct Material {
    std::array<float, 3> ambient{};
    std::array<float, 3> diffuse{};
    std::array<float, 3> specular{};
    float shininess = 1.0f;
};

struct DrawableObject {
    std::vector<float> interleaved; // position.xyz, normal.xyz
    Material material;
    GLsizei vertex_count = 0;
    GLuint vao = 0;
    GLuint vbo = 0;
};

Scene g_scene;
std::vector<DrawableObject> g_drawables;
Arcball g_arcball;
int g_window_width = 800;
int g_window_height = 800;
int g_shading_mode = 0;

GLuint g_shader_program = 0;

struct UniformLocations {
    GLint model_view;
    GLint projection;
    GLint normal_matrix;
    GLint scene_ambient;
    GLint material_ambient;
    GLint material_diffuse;
    GLint material_specular;
    GLint material_shininess;
    GLint num_lights;
    GLint light_positions;
    GLint light_colors;
    GLint light_attenuation;
    GLint shading_mode;
} g_uniforms;

struct AttributeLocations {
    GLint position;
    GLint normal;
} g_attributes;

} // namespace

namespace {

const char* kVertexShader = R"GLSL(
#version 120

const int MAX_LIGHTS = 8;

attribute vec3 inPosition;
attribute vec3 inNormal;

uniform mat4 uModelView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;
uniform int uMode;
uniform vec3 uSceneAmbient;
uniform vec3 uMaterialAmbient;
uniform vec3 uMaterialDiffuse;
uniform vec3 uMaterialSpecular;
uniform float uMaterialShininess;
uniform int uNumLights;
uniform vec3 uLightPosition[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];
uniform float uLightAttenuation[MAX_LIGHTS];

varying vec3 vNormal;
varying vec3 vPosition;
varying vec3 vColor;

vec3 computeLighting(vec3 position, vec3 normal) {
    vec3 ambient = uSceneAmbient * uMaterialAmbient;
    vec3 viewDir = normalize(-position);
    vec3 color = ambient;
    for (int i = 0; i < MAX_LIGHTS; ++i) {
        if (i >= uNumLights) break;
        vec3 lightVector = uLightPosition[i] - position;
        float distance = length(lightVector);
        vec3 L = lightVector / max(distance, 1e-5);
        float attenuation = 1.0 / (1.0 + uLightAttenuation[i] * distance * distance);
        float ndotl = max(dot(normal, L), 0.0);
        vec3 diffuse = uMaterialDiffuse * uLightColor[i] * ndotl;
        vec3 specular = vec3(0.0);
        if (ndotl > 0.0) {
            vec3 R = reflect(-L, normal);
            float specAngle = max(dot(R, viewDir), 0.0);
            specular = uMaterialSpecular * uLightColor[i] * pow(specAngle, uMaterialShininess);
        }
        color += attenuation * (diffuse + specular);
    }
    return color;
}

void main() {
    vec4 viewPos4 = uModelView * vec4(inPosition, 1.0);
    vec3 viewPos = viewPos4.xyz / viewPos4.w;
    vec3 normal = normalize(uNormalMatrix * inNormal);

    if (uMode == 0) {
        vColor = computeLighting(viewPos, normal);
    } else {
        vColor = vec3(0.0);
    }
    vNormal = normal;
    vPosition = viewPos;
    gl_Position = uProjection * vec4(viewPos, 1.0);
}
)GLSL";

const char* kFragmentShader = R"GLSL(
#version 120

const int MAX_LIGHTS = 8;

uniform int uMode;
uniform vec3 uSceneAmbient;
uniform vec3 uMaterialAmbient;
uniform vec3 uMaterialDiffuse;
uniform vec3 uMaterialSpecular;
uniform float uMaterialShininess;
uniform int uNumLights;
uniform vec3 uLightPosition[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];
uniform float uLightAttenuation[MAX_LIGHTS];

varying vec3 vNormal;
varying vec3 vPosition;
varying vec3 vColor;

vec3 computeLighting(vec3 position, vec3 normal) {
    vec3 ambient = uSceneAmbient * uMaterialAmbient;
    vec3 viewDir = normalize(-position);
    vec3 color = ambient;
    for (int i = 0; i < MAX_LIGHTS; ++i) {
        if (i >= uNumLights) break;
        vec3 lightVector = uLightPosition[i] - position;
        float distance = length(lightVector);
        vec3 L = lightVector / max(distance, 1e-5);
        float attenuation = 1.0 / (1.0 + uLightAttenuation[i] * distance * distance);
        float ndotl = max(dot(normal, L), 0.0);
        vec3 diffuse = uMaterialDiffuse * uLightColor[i] * ndotl;
        vec3 specular = vec3(0.0);
        if (ndotl > 0.0) {
            vec3 R = reflect(-L, normal);
            float specAngle = max(dot(R, viewDir), 0.0);
            specular = uMaterialSpecular * uLightColor[i] * pow(specAngle, uMaterialShininess);
        }
        color += attenuation * (diffuse + specular);
    }
    return color;
}

void main() {
    vec3 color;
    if (uMode == 0) {
        color = vColor;
    } else {
        vec3 normal = normalize(vNormal);
        color = computeLighting(vPosition, normal);
    }
    gl_FragColor = vec4(color, 1.0);
}
)GLSL";

} // namespace

void build_drawables() {
    g_drawables.clear();
    g_drawables.reserve(g_scene.scene_objects.size());

    for (const auto& inst : g_scene.scene_objects) {
        DrawableObject drawable;

        auto copy_vec3 = [](const Eigen::Vector3d& src, std::array<float, 3>& dst) {
            dst[0] = static_cast<float>(src[0]);
            dst[1] = static_cast<float>(src[1]);
            dst[2] = static_cast<float>(src[2]);
        };
        copy_vec3(inst.ambient, drawable.material.ambient);
        copy_vec3(inst.diffuse, drawable.material.diffuse);
        copy_vec3(inst.specular, drawable.material.specular);
        drawable.material.shininess = static_cast<float>(std::max(0.0, std::min(inst.shininess, 128.0)));

        drawable.interleaved.reserve(inst.obj.faces.size() * 3 * 6);
        for (const auto& face : inst.obj.faces) {
            const Vertex& v1 = inst.obj.vertices.at(face.v1);
            const Vertex& v2 = inst.obj.vertices.at(face.v2);
            const Vertex& v3 = inst.obj.vertices.at(face.v3);
            const Normal& n1 = inst.obj.normals.at(face.vn1);
            const Normal& n2 = inst.obj.normals.at(face.vn2);
            const Normal& n3 = inst.obj.normals.at(face.vn3);

            const std::array<const Vertex*, 3> verts{&v1, &v2, &v3};
            const std::array<const Normal*, 3> norms{&n1, &n2, &n3};

            for (int i = 0; i < 3; ++i) {
                drawable.interleaved.push_back(static_cast<float>(verts[i]->x));
                drawable.interleaved.push_back(static_cast<float>(verts[i]->y));
                drawable.interleaved.push_back(static_cast<float>(verts[i]->z));
                drawable.interleaved.push_back(static_cast<float>(norms[i]->x));
                drawable.interleaved.push_back(static_cast<float>(norms[i]->y));
                drawable.interleaved.push_back(static_cast<float>(norms[i]->z));
            }
        }
        drawable.vertex_count = static_cast<GLsizei>(drawable.interleaved.size() / 6);
        g_drawables.push_back(std::move(drawable));
    }
}

void upload_drawables() {
    for (auto& drawable : g_drawables) {
        if (drawable.vao == 0) {
            glGenVertexArrays(1, &drawable.vao);
        }
        if (drawable.vbo == 0) {
            glGenBuffers(1, &drawable.vbo);
        }
        glBindVertexArray(drawable.vao);
        glBindBuffer(GL_ARRAY_BUFFER, drawable.vbo);
        glBufferData(GL_ARRAY_BUFFER, drawable.interleaved.size() * sizeof(float), drawable.interleaved.data(), GL_STATIC_DRAW);

        const GLsizei stride = static_cast<GLsizei>(6 * sizeof(float));
        glEnableVertexAttribArray(static_cast<GLuint>(g_attributes.position));
        glVertexAttribPointer(static_cast<GLuint>(g_attributes.position), 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(0));
        glEnableVertexAttribArray(static_cast<GLuint>(g_attributes.normal));
        glVertexAttribPointer(static_cast<GLuint>(g_attributes.normal), 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(3 * sizeof(float)));
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void init_lights_uniforms(const Eigen::Matrix4f& model_view) {
    const auto& lights = g_scene.lights;
    const int num_lights = std::min(static_cast<int>(lights.size()), kMaxLights);

    std::vector<float> positions(3 * kMaxLights, 0.0f);
    std::vector<float> colors(3 * kMaxLights, 0.0f);
    std::vector<float> attenuation(kMaxLights, 0.0f);

    for (int i = 0; i < num_lights; ++i) {
        const auto& light = lights[i];
        Eigen::Vector4f light_pos(static_cast<float>(light.x), static_cast<float>(light.y), static_cast<float>(light.z), 1.0f);
        Eigen::Vector4f view_pos = model_view * light_pos;
        positions[3 * i + 0] = view_pos[0];
        positions[3 * i + 1] = view_pos[1];
        positions[3 * i + 2] = view_pos[2];

        colors[3 * i + 0] = static_cast<float>(light.r);
        colors[3 * i + 1] = static_cast<float>(light.g);
        colors[3 * i + 2] = static_cast<float>(light.b);

        attenuation[i] = static_cast<float>(light.atten);
    }

    glUniform1i(g_uniforms.num_lights, num_lights);
    glUniform3fv(g_uniforms.light_positions, kMaxLights, positions.data());
    glUniform3fv(g_uniforms.light_colors, kMaxLights, colors.data());
    glUniform1fv(g_uniforms.light_attenuation, kMaxLights, attenuation.data());
}

Eigen::Matrix4f to_matrix4f(const Eigen::Matrix4d& m) {
    Eigen::Matrix4f out;
    out = m.cast<float>();
    return out;
}

Eigen::Matrix4d arcball_matrix() {
    const auto raw = g_arcball.rotation().to_matrix();
    Eigen::Matrix4d m;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            m(row, col) = raw[col * 4 + row];
        }
    }
    return m;
}

Eigen::Matrix3f compute_normal_matrix(const Eigen::Matrix4f& model_view) {
    Eigen::Matrix3f upper = model_view.block<3, 3>(0, 0);
    return upper.inverse().transpose();
}

void init_gl_state() {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void init_shader() {
    g_shader_program = build_shader_program(kVertexShader, kFragmentShader);
    glUseProgram(g_shader_program);

    g_attributes.position = glGetAttribLocation(g_shader_program, "inPosition");
    g_attributes.normal = glGetAttribLocation(g_shader_program, "inNormal");

    g_uniforms.model_view = glGetUniformLocation(g_shader_program, "uModelView");
    g_uniforms.projection = glGetUniformLocation(g_shader_program, "uProjection");
    g_uniforms.normal_matrix = glGetUniformLocation(g_shader_program, "uNormalMatrix");
    g_uniforms.scene_ambient = glGetUniformLocation(g_shader_program, "uSceneAmbient");
    g_uniforms.material_ambient = glGetUniformLocation(g_shader_program, "uMaterialAmbient");
    g_uniforms.material_diffuse = glGetUniformLocation(g_shader_program, "uMaterialDiffuse");
    g_uniforms.material_specular = glGetUniformLocation(g_shader_program, "uMaterialSpecular");
    g_uniforms.material_shininess = glGetUniformLocation(g_shader_program, "uMaterialShininess");
    g_uniforms.num_lights = glGetUniformLocation(g_shader_program, "uNumLights");
    g_uniforms.light_positions = glGetUniformLocation(g_shader_program, "uLightPosition");
    g_uniforms.light_colors = glGetUniformLocation(g_shader_program, "uLightColor");
    g_uniforms.light_attenuation = glGetUniformLocation(g_shader_program, "uLightAttenuation");
    g_uniforms.shading_mode = glGetUniformLocation(g_shader_program, "uMode");

    if (g_attributes.position < 0 || g_attributes.normal < 0) {
        throw std::runtime_error("Failed to locate vertex attributes");
    }

    glUniform3f(g_uniforms.scene_ambient, 1.0f, 1.0f, 1.0f);
    glUniform1i(g_uniforms.shading_mode, g_shading_mode);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Eigen::Matrix4d arcball = arcball_matrix();
    Eigen::Matrix4d model_view_d = g_scene.cam_transforms.Cinv * arcball;
    Eigen::Matrix4f model_view = to_matrix4f(model_view_d);
    Eigen::Matrix4f projection = to_matrix4f(g_scene.cam_transforms.P);
    Eigen::Matrix3f normal_matrix = compute_normal_matrix(model_view);

    glUseProgram(g_shader_program);
    glUniformMatrix4fv(g_uniforms.model_view, 1, GL_FALSE, model_view.data());
    glUniformMatrix4fv(g_uniforms.projection, 1, GL_FALSE, projection.data());
    glUniformMatrix3fv(g_uniforms.normal_matrix, 1, GL_FALSE, normal_matrix.data());
    glUniform1i(g_uniforms.shading_mode, g_shading_mode);

    init_lights_uniforms(model_view);

    for (const auto& drawable : g_drawables) {
        glUniform3fv(g_uniforms.material_ambient, 1, drawable.material.ambient.data());
        glUniform3fv(g_uniforms.material_diffuse, 1, drawable.material.diffuse.data());
        glUniform3fv(g_uniforms.material_specular, 1, drawable.material.specular.data());
        glUniform1f(g_uniforms.material_shininess, drawable.material.shininess);

        glBindVertexArray(drawable.vao);
        glDrawArrays(GL_TRIANGLES, 0, drawable.vertex_count);
    }
    glBindVertexArray(0);

    glutSwapBuffers();
}

static double camera_aspect_from_P(const Eigen::Matrix4d& P) {
    return P(1, 1) / P(0, 0);
}

void apply_letterboxed_viewport(int win_w, int win_h) {
    const double A_cam = camera_aspect_from_P(g_scene.cam_transforms.P);
    const double A_win = static_cast<double>(win_w) / static_cast<double>(win_h);

    int vx = 0, vy = 0, vw = win_w, vh = win_h;
    if (A_win > A_cam) {
        vw = static_cast<int>(std::round(vh * A_cam));
        vx = (win_w - vw) / 2;
    } else if (A_win < A_cam) {
        vh = static_cast<int>(std::round(vw / A_cam));
        vy = (win_h - vh) / 2;
    }
    glViewport(vx, vy, vw, vh);
    g_arcball.set_viewport(vx, vy, vw, vh);
}

void reshape(int width, int height) {
    g_window_width = std::max(width, 1);
    g_window_height = std::max(height, 1);
    apply_letterboxed_viewport(g_window_width, g_window_height);
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

void initialize_glut(int* argc, char** argv) {
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_window_width, g_window_height);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("OpenGL Scene Renderer");
}

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres] [mode]\n";
        return 1;
    }

    g_shading_mode = std::stoi(argv[4]);
    if (g_shading_mode != 0 && g_shading_mode != 1) {
        std::cerr << "Mode must be 0 (Gouraud) or 1 (Phong)\n";
        return 1;
    }

    const std::size_t xres = parse_size_t(argv[2]);
    const std::size_t yres = parse_size_t(argv[3]);

    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cerr << "Could not open file: " << argv[1] << "\n";
        return 1;
    }

    try {
        g_scene = parse_scene_file(fin, parse_parent_path(argv[1]));
    } catch (const std::exception& e) {
        std::cerr << "Error parsing scene: " << e.what() << "\n";
        return 1;
    }

    g_window_width = static_cast<int>(xres);
    g_window_height = static_cast<int>(yres);
    g_arcball.set_window(g_window_width, g_window_height);

    build_drawables();

    initialize_glut(&argc, argv);

    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "GLEW init error: " << glewGetErrorString(err) << "\n";
        return 1;
    }

    init_gl_state();
    init_shader();
    upload_drawables();

    apply_letterboxed_viewport(g_window_width, g_window_height);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
