#include "scene_loader.h"
#include "arcball.h"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

// Variant of our object class compatible with OpenGL (all vertices laid out, including repeats)s
struct DrawableObject {
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> normals;
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat shininess;
};

namespace {
Scene g_scene;
std::vector<DrawableObject> g_drawables;
Arcball g_arcball;
int g_window_width = 800;
int g_window_height = 800;
}

void build_drawables() {
    // Created OpenGL compatible structs from our parsed objects

    g_drawables.clear();
    g_drawables.reserve(g_scene.scene_objects.size());

    for (const auto& inst : g_scene.scene_objects) {
        DrawableObject drawable;
        // 3 verts and normals per face, each made of 3 floats
        drawable.vertices.reserve(inst.obj.faces.size() * 3 * 3);
        drawable.normals.reserve(inst.obj.faces.size() * 3 * 3);

        auto fill_material = [&](const Eigen::Vector3d& src, GLfloat out[4]) {
            // Simple helper to copy over data into new structs
            out[0] = static_cast<GLfloat>(src[0]);
            out[1] = static_cast<GLfloat>(src[1]);
            out[2] = static_cast<GLfloat>(src[2]);
            out[3] = 1.0f;
        };
        fill_material(inst.ambient, drawable.ambient);
        fill_material(inst.diffuse, drawable.diffuse);
        fill_material(inst.specular, drawable.specular);
        double shininess = std::max(0.0, std::min(inst.shininess, 128.0));
        drawable.shininess = static_cast<GLfloat>(shininess);

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
                drawable.normals.push_back(static_cast<GLfloat>(norms[i]->x));
                drawable.normals.push_back(static_cast<GLfloat>(norms[i]->y));
                drawable.normals.push_back(static_cast<GLfloat>(norms[i]->z));

                drawable.vertices.push_back(static_cast<GLfloat>(verts[i]->x));
                drawable.vertices.push_back(static_cast<GLfloat>(verts[i]->y));
                drawable.vertices.push_back(static_cast<GLfloat>(verts[i]->z));
            }
        }

        g_drawables.push_back(std::move(drawable));
    }
}

void init_lights() {
    glEnable(GL_LIGHTING);
    // More realistic lighting -- the viewer isn't infinitely far away
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

    const auto& lights = g_scene.lights;
    GLint max_lights = 0;
    glGetIntegerv(GL_MAX_LIGHTS, &max_lights);
    GLfloat zero4[4]  = { 0.f, 0.f, 0.f, 1.f };

    for (GLint i = 0; i < max_lights; ++i) {
        GLenum light_id = static_cast<GLenum>(GL_LIGHT0 + i);
        if (i < static_cast<GLint>(lights.size())) {
            const auto& light = lights[i];
            glEnable(light_id);
            GLfloat color[4] = {
                static_cast<GLfloat>(light.r),
                static_cast<GLfloat>(light.g),
                static_cast<GLfloat>(light.b),
                1.0f
            };
            glLightfv(light_id, GL_DIFFUSE, color);
            glLightfv(light_id, GL_SPECULAR, color);
            glLightfv(light_id, GL_AMBIENT, zero4);
            glLightf(light_id, GL_QUADRATIC_ATTENUATION, static_cast<GLfloat>(light.atten));
        } else {
            glDisable(light_id);
        }
    }

    // Activate global ambient lighting for all objects
    GLfloat color[4] = {1.f, 1.f, 1.f, 1.f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, color);
}

void init_gl() {
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Default bground is black

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(g_scene.cam_transforms.P.data());

    glMatrixMode(GL_MODELVIEW);
    init_lights();
}

void set_lights() {
    // Call every draw loop, sets the light positions after updating GL_MODELVIEW
    // That way the lights are rotated by the arcball
    const auto& lights = g_scene.lights;
    int num_lights = lights.size();
    
    for(int i = 0; i < num_lights; ++i)
    {
        const auto& light = lights[i];
        GLfloat position[4] = {
            static_cast<GLfloat>(light.x),
            static_cast<GLfloat>(light.y),
            static_cast<GLfloat>(light.z),
            1.0f
        };
        int light_id = GL_LIGHT0 + i;
        glLightfv(light_id, GL_POSITION, position);
    }
}

void draw_scene() {
    for (const auto& drawable : g_drawables) {
        // No transforms to apply -- our objects are already transformed during parsing
        glMaterialfv(GL_FRONT, GL_AMBIENT, drawable.ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, drawable.diffuse);
        glMaterialfv(GL_FRONT, GL_SPECULAR, drawable.specular);
        glMaterialf(GL_FRONT, GL_SHININESS, drawable.shininess);

        glVertexPointer(3, GL_FLOAT, 0, drawable.vertices.data());
        glNormalPointer(GL_FLOAT, 0, drawable.normals.data());
        const GLsizei vertexCount = static_cast<GLsizei>(drawable.vertices.size() / 3);
        glDrawArrays(GL_TRIANGLES, 0, vertexCount);
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    glMultMatrixd(g_scene.cam_transforms.Cinv.data());

    auto arcball_matrix = g_arcball.rotation().to_matrix();
    glMultMatrixd(arcball_matrix.data());

    set_lights();
    draw_scene();

    glutSwapBuffers();
}


// Gets the actual aspect ratio defined by our camera
static double camera_aspect_from_P(const Eigen::Matrix4d& P) {
    return P(1,1) / P(0,0); // Essentially (r-l)/(t-b) or width/height
}

// I found this code online, but it helped to make my renderings look
// properly proportioned on the screen at any window size.
// It essentially calculates if it needs to pad the window at all in order
// to match the rendering to the necessary aspect ratio, instead of warping things
void apply_letterboxed_viewport(int win_w, int win_h) {
    const double A_cam = camera_aspect_from_P(g_scene.cam_transforms.P);
    const double A_win = (double)win_w / (double)win_h;

    int vx = 0, vy = 0, vw = win_w, vh = win_h;
    if (A_win > A_cam) {
        // window is wider -> pillarbox left/right
        vw = (int)std::round(vh * A_cam);
        vx = (win_w - vw) / 2;
    } else if (A_win < A_cam) {
        // window is taller -> letterbox top/bottom
        vh = (int)std::round(vw / A_cam);
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

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres]\n";
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

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_window_width, g_window_height);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("OpenGL Scene Renderer");

    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "GLEW init error: " << glewGetErrorString(err) << "\n";
        return 1;
    }

    init_gl();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}

