#include "arcball.h"
#include "png_loader.h"
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
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
Arcball g_arcball;
int g_window_width = 800;
int g_window_height = 800;

GLuint g_shader_program = 0;
GLuint g_vertex_array = 0;
GLuint g_vertex_buffer = 0;
GLuint g_color_texture = 0;
GLuint g_normal_texture = 0;

struct UniformLocations {
    GLint model_view;
    GLint projection;
    GLint normal_matrix;
    GLint light_position;
    GLint camera_position;
    GLint color_sampler;
    GLint normal_sampler;
} g_uniforms;

struct AttributeLocations {
    GLint position;
    GLint normal;
    GLint tangent;
    GLint texcoord;
} g_attributes;

} // namespace

namespace {
const char* kVertexShader = R"GLSL(
#version 120

attribute vec3 inPosition;
attribute vec3 inNormal;
attribute vec3 inTangent;
attribute vec2 inTexCoord;

uniform mat4 uModelView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vTangent;
varying vec3 vBitangent;
varying vec2 vTexCoord;

void main() {
    vec4 viewPos = uModelView * vec4(inPosition, 1.0);
    vec3 normal = normalize(uNormalMatrix * inNormal);
    vec3 tangent = normalize(uNormalMatrix * inTangent);
    vec3 bitangent = normalize(cross(normal, tangent));

    vPosition = viewPos.xyz;
    vNormal = normal;
    vTangent = tangent;
    vBitangent = bitangent;
    vTexCoord = inTexCoord;

    gl_Position = uProjection * viewPos;
}
)GLSL";

const char* kFragmentShader = R"GLSL(
#version 120

uniform vec3 uLightPosition;
uniform vec3 uCameraPosition;
uniform sampler2D uColorTexture;
uniform sampler2D uNormalTexture;

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vTangent;
varying vec3 vBitangent;
varying vec2 vTexCoord;

vec3 computeLighting(vec3 position, vec3 normal, vec3 albedo) {
    vec3 lightVector = uLightPosition - position;
    float distance = length(lightVector);
    vec3 L = lightVector / max(distance, 1e-5);
    vec3 V = normalize(uCameraPosition - position);
    vec3 H = normalize(L + V);

    float ndotl = max(dot(normal, L), 0.0);
    float ndoth = max(dot(normal, H), 0.0);

    vec3 ambient = 0.1 * albedo;
    vec3 diffuse = albedo * ndotl;
    vec3 specular = vec3(0.3) * pow(ndoth, 32.0);

    return ambient + diffuse + specular;
}

void main() {
    vec3 normalSample = texture2D(uNormalTexture, vTexCoord).rgb;
    normalSample = normalize(normalSample * 2.0 - 1.0);
    mat3 TBN = mat3(normalize(vTangent), normalize(vBitangent), normalize(vNormal));
    vec3 normal = normalize(TBN * normalSample);

    vec3 albedo = texture2D(uColorTexture, vTexCoord).rgb;
    vec3 color = computeLighting(vPosition, normal, albedo);
    gl_FragColor = vec4(color, 1.0);
}
)GLSL";

} // namespace

Eigen::Matrix4f arcball_matrix() {
    const auto raw = g_arcball.rotation().to_matrix();
    Eigen::Matrix4f m;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            m(row, col) = static_cast<float>(raw[col * 4 + row]);
        }
    }
    return m;
}

struct Vertex {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector3f tangent;
    Eigen::Vector2f texcoord;
};

std::vector<Vertex> build_quad_geometry() {
    std::vector<Vertex> vertices(6);

    const std::array<Eigen::Vector3f, 6> positions = {
        Eigen::Vector3f(-1.0f, -1.0f, 0.0f),
        Eigen::Vector3f(1.0f, -1.0f, 0.0f),
        Eigen::Vector3f(1.0f, 1.0f, 0.0f),
        Eigen::Vector3f(-1.0f, -1.0f, 0.0f),
        Eigen::Vector3f(1.0f, 1.0f, 0.0f),
        Eigen::Vector3f(-1.0f, 1.0f, 0.0f)
    };

    const std::array<Eigen::Vector2f, 6> texcoords = {
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(1.0f, 0.0f),
        Eigen::Vector2f(1.0f, 1.0f),
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(1.0f, 1.0f),
        Eigen::Vector2f(0.0f, 1.0f)
    };

    Eigen::Vector3f normal(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f tangent(1.0f, 0.0f, 0.0f);

    for (std::size_t i = 0; i < vertices.size(); ++i) {
        vertices[i].position = positions[i];
        vertices[i].normal = normal;
        vertices[i].tangent = tangent;
        vertices[i].texcoord = texcoords[i];
    }
    return vertices;
}

GLuint create_texture_from_png(const std::string& filename) {
    PngData data = load_png_rgba(filename);

    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, data.width, data.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.pixels.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
}

void upload_geometry(const std::vector<Vertex>& vertices) {
    if (g_vertex_array == 0) {
        glGenVertexArrays(1, &g_vertex_array);
    }
    if (g_vertex_buffer == 0) {
        glGenBuffers(1, &g_vertex_buffer);
    }

    glBindVertexArray(g_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, g_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    const GLsizei stride = sizeof(Vertex);
    glEnableVertexAttribArray(static_cast<GLuint>(g_attributes.position));
    glVertexAttribPointer(static_cast<GLuint>(g_attributes.position), 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offsetof(Vertex, position)));

    glEnableVertexAttribArray(static_cast<GLuint>(g_attributes.normal));
    glVertexAttribPointer(static_cast<GLuint>(g_attributes.normal), 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offsetof(Vertex, normal)));

    glEnableVertexAttribArray(static_cast<GLuint>(g_attributes.tangent));
    glVertexAttribPointer(static_cast<GLuint>(g_attributes.tangent), 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offsetof(Vertex, tangent)));

    glEnableVertexAttribArray(static_cast<GLuint>(g_attributes.texcoord));
    glVertexAttribPointer(static_cast<GLuint>(g_attributes.texcoord), 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offsetof(Vertex, texcoord)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Eigen::Matrix4f look_at(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up) {
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 0) = s.x();
    result(0, 1) = s.y();
    result(0, 2) = s.z();
    result(1, 0) = u.x();
    result(1, 1) = u.y();
    result(1, 2) = u.z();
    result(2, 0) = -f.x();
    result(2, 1) = -f.y();
    result(2, 2) = -f.z();
    result(0, 3) = -s.dot(eye);
    result(1, 3) = -u.dot(eye);
    result(2, 3) = f.dot(eye);
    return result;
}

Eigen::Matrix4f perspective(float fovy_deg, float aspect, float znear, float zfar) {
    const float kPi = 3.14159265358979323846f;
    const float fovy_rad = fovy_deg * kPi / 180.0f;
    const float f = 1.0f / std::tan(fovy_rad / 2.0f);
    Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
    result(0, 0) = f / aspect;
    result(1, 1) = f;
    result(2, 2) = (zfar + znear) / (znear - zfar);
    result(2, 3) = (2.0f * zfar * znear) / (znear - zfar);
    result(3, 2) = -1.0f;
    return result;
}

Eigen::Matrix3f compute_normal_matrix(const Eigen::Matrix4f& model_view) {
    return model_view.block<3, 3>(0, 0).inverse().transpose();
}

void init_shader() {
    g_shader_program = build_shader_program(kVertexShader, kFragmentShader);
    glUseProgram(g_shader_program);

    g_attributes.position = glGetAttribLocation(g_shader_program, "inPosition");
    g_attributes.normal = glGetAttribLocation(g_shader_program, "inNormal");
    g_attributes.tangent = glGetAttribLocation(g_shader_program, "inTangent");
    g_attributes.texcoord = glGetAttribLocation(g_shader_program, "inTexCoord");

    if (g_attributes.position < 0 || g_attributes.normal < 0 || g_attributes.tangent < 0 || g_attributes.texcoord < 0) {
        throw std::runtime_error("Failed to get attribute locations");
    }

    g_uniforms.model_view = glGetUniformLocation(g_shader_program, "uModelView");
    g_uniforms.projection = glGetUniformLocation(g_shader_program, "uProjection");
    g_uniforms.normal_matrix = glGetUniformLocation(g_shader_program, "uNormalMatrix");
    g_uniforms.light_position = glGetUniformLocation(g_shader_program, "uLightPosition");
    g_uniforms.camera_position = glGetUniformLocation(g_shader_program, "uCameraPosition");
    g_uniforms.color_sampler = glGetUniformLocation(g_shader_program, "uColorTexture");
    g_uniforms.normal_sampler = glGetUniformLocation(g_shader_program, "uNormalTexture");

    glUniform1i(g_uniforms.color_sampler, 0);
    glUniform1i(g_uniforms.normal_sampler, 1);
}

void init_gl_state() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Eigen::Matrix4f view = look_at(Eigen::Vector3f(0.0f, 0.0f, 3.0f), Eigen::Vector3f::Zero(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    Eigen::Matrix4f model = arcball_matrix();
    Eigen::Matrix4f model_view = view * model;
    Eigen::Matrix4f proj = perspective(45.0f, static_cast<float>(g_window_width) / static_cast<float>(g_window_height), 0.1f, 100.0f);
    Eigen::Matrix3f normal_matrix = compute_normal_matrix(model_view);

    glUseProgram(g_shader_program);
    glUniformMatrix4fv(g_uniforms.model_view, 1, GL_FALSE, model_view.data());
    glUniformMatrix4fv(g_uniforms.projection, 1, GL_FALSE, proj.data());
    glUniformMatrix3fv(g_uniforms.normal_matrix, 1, GL_FALSE, normal_matrix.data());

    Eigen::Vector3f light_pos_view = (view * Eigen::Vector4f(1.5f, 1.5f, 3.0f, 1.0f)).head<3>();
    glUniform3fv(g_uniforms.light_position, 1, light_pos_view.data());
    glUniform3f(g_uniforms.camera_position, 0.0f, 0.0f, 0.0f);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g_color_texture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, g_normal_texture);

    glBindVertexArray(g_vertex_array);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glutSwapBuffers();
}

void reshape(int width, int height) {
    g_window_width = std::max(width, 1);
    g_window_height = std::max(height, 1);
    g_arcball.set_window(g_window_width, g_window_height);
    g_arcball.set_viewport(0, 0, g_window_width, g_window_height);
    glViewport(0, 0, g_window_width, g_window_height);
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
    glutCreateWindow("Normal Map Renderer");
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " [color_texture.png] [normal_map.png]\n";
        return 1;
    }

    initialize_glut(&argc, argv);

    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "GLEW init error: " << glewGetErrorString(err) << "\n";
        return 1;
    }

    try {
        init_gl_state();
        init_shader();

        g_color_texture = create_texture_from_png(argv[1]);
        g_normal_texture = create_texture_from_png(argv[2]);

        auto vertices = build_quad_geometry();
        upload_geometry(vertices);

        g_arcball.set_window(g_window_width, g_window_height);
        g_arcball.set_viewport(0, 0, g_window_width, g_window_height);
    } catch (const std::exception& e) {
        std::cerr << "Initialization error: " << e.what() << "\n";
        return 1;
    }

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
