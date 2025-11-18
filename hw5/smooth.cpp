#include "scene_loader.h"
#include "arcball.h"
#include "halfedge.h"

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
#include <Eigen/Sparse>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

struct MeshGeometry {
    Object obj;
    std::vector<Vertex*> vertex_ptrs;
    std::vector<Face*> face_ptrs;
    std::vector<HEV*> hevs;
    std::vector<HEF*> hefs;
    std::vector<Vec3f> vertex_normals; // 1-indexed to match vertex order
};

struct RenderObject {
    MeshGeometry mesh;
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat shininess;
};

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
std::vector<RenderObject> g_render_objects;
std::vector<DrawableObject> g_drawables;
Arcball g_arcball;
int g_window_width = 800;
int g_window_height = 800;
double g_time_step = 0.0;

Eigen::Vector3d hev_pos(const HEV* v) {
    return Eigen::Vector3d(v->x, v->y, v->z);
}

Vec3f calc_face_normal(const HEF* face) {
    const HE* e0 = face->edge;
    const HE* e1 = face->edge->next;
    const HE* e2 = face->edge->next->next;

    Eigen::Vector3d p0 = hev_pos(e0->vertex);
    Eigen::Vector3d p1 = hev_pos(e1->vertex);
    Eigen::Vector3d p2 = hev_pos(e2->vertex);

    Eigen::Vector3d n = (p1 - p0).cross(p2 - p0);
    if (n.norm() > 0.0) {
        n.normalize();
    }
    return Vec3f{static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z())};
}

double calc_area(const HEF* face) {
    const HE* e0 = face->edge;
    const HE* e1 = face->edge->next;
    const HE* e2 = face->edge->next->next;

    Eigen::Vector3d p0 = hev_pos(e0->vertex);
    Eigen::Vector3d p1 = hev_pos(e1->vertex);
    Eigen::Vector3d p2 = hev_pos(e2->vertex);

    return 0.5 * ((p1 - p0).cross(p2 - p0)).norm();
}

void compute_vertex_normals(MeshGeometry& mesh) {
    mesh.vertex_normals.assign(mesh.hevs.size(), Vec3f{0.0f, 0.0f, 0.0f});

    for (std::size_t i = 1; i < mesh.hevs.size(); ++i) {
        HEV* v = mesh.hevs[i];
        if (v->out == nullptr) continue;

        Eigen::Vector3d accum = Eigen::Vector3d::Zero();
        HE* he_start = v->out;
        HE* he = he_start;
        do {
            Vec3f face_normal = calc_face_normal(he->face);
            Eigen::Vector3d n(face_normal.x, face_normal.y, face_normal.z);
            double area = calc_area(he->face);
            accum += area * n;
            he = he->flip->next;
        } while (he != he_start);

        if (accum.norm() > 0.0) {
            accum.normalize();
        }
        v->normal = Vec3f{static_cast<float>(accum.x()), static_cast<float>(accum.y()), static_cast<float>(accum.z())};
        mesh.vertex_normals[i] = v->normal;
    }
}

double calc_cotangent(const HE* edge) {
    const HEV* v0 = edge->vertex;
    const HEV* v1 = edge->next->vertex;
    const HEV* v2 = edge->next->next->vertex; // vertex opposite the edge

    Eigen::Vector3d a = hev_pos(v0) - hev_pos(v2);
    Eigen::Vector3d b = hev_pos(v1) - hev_pos(v2);
    Eigen::Vector3d cross = a.cross(b);
    double sin_theta = cross.norm();
    if (sin_theta < 1e-12) return 0.0;
    return a.dot(b) / sin_theta;
}

double vertex_mixed_area(const HEV* v) {
    if (v->out == nullptr) return 0.0;
    double area = 0.0;
    HE* he_start = v->out;
    HE* he = he_start;
    do {
        area += calc_area(he->face) / 3.0;
        he = he->flip->next;
    } while (he != he_start);
    return area;
}

Eigen::SparseMatrix<double> build_fairing_matrix(const MeshGeometry& mesh, double h) {
    const std::size_t n = mesh.hevs.size() - 1;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n * 7); // heuristic

    for (std::size_t i = 1; i < mesh.hevs.size(); ++i) {
        HEV* v = mesh.hevs[i];
        int row = v->index;
        if (v->out == nullptr) {
            triplets.emplace_back(row, row, 1.0);
            continue;
        }

        double area = vertex_mixed_area(v);
        if (std::abs(area) < 1e-12) {
            triplets.emplace_back(row, row, 1.0);
            continue;
        }

        double weight_sum = 0.0;
        HE* he_start = v->out;
        HE* he = he_start;
        do {
            HEV* neighbor = he->next->vertex;
            double cot1 = calc_cotangent(he);
            double cot2 = (he->flip != nullptr) ? calc_cotangent(he->flip) : 0.0;
            double w = cot1 + cot2;
            weight_sum += w;

            // Using (I - h * Δ) with the cotangent Laplacian Δ = (1/(2A)) Σ (cot α + cot β)(x_j - x_i)
            // so off-diagonals become -h * w / (2A) and the diagonal accumulates +h * Σw / (2A).
            double coeff = -h * (w / (2.0 * area));
            if (coeff != 0.0) {
                triplets.emplace_back(row, neighbor->index, coeff);
            }
            he = he->flip->next;
        } while (he != he_start);

        double diag = 1.0 + h * (weight_sum / (2.0 * area));
        triplets.emplace_back(row, row, diag);
    }

    Eigen::SparseMatrix<double> F(static_cast<int>(n), static_cast<int>(n));
    F.setFromTriplets(triplets.begin(), triplets.end());
    return F;
}

void update_mesh_from_solution(MeshGeometry& mesh,
                               const Eigen::VectorXd& x,
                               const Eigen::VectorXd& y,
                               const Eigen::VectorXd& z) {
    for (std::size_t i = 1; i < mesh.hevs.size(); ++i) {
        HEV* v = mesh.hevs[i];
        int idx = v->index;
        v->x = x[idx];
        v->y = y[idx];
        v->z = z[idx];

        // keep the original vertex list in sync for rendering
        mesh.obj.vertices[i].x = v->x;
        mesh.obj.vertices[i].y = v->y;
        mesh.obj.vertices[i].z = v->z;
    }
}

void apply_implicit_fairing(RenderObject& obj, double h) {
    if (obj.mesh.hevs.size() <= 1) return;

    Eigen::SparseMatrix<double> F = build_fairing_matrix(obj.mesh, h);

    const std::size_t n = obj.mesh.hevs.size() - 1;
    Eigen::VectorXd x0(n), y0(n), z0(n);
    for (std::size_t i = 1; i < obj.mesh.hevs.size(); ++i) {
        const HEV* v = obj.mesh.hevs[i];
        int idx = v->index;
        x0[idx] = v->x;
        y0[idx] = v->y;
        z0[idx] = v->z;
    }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(F);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Failed to factorize fairing matrix");
    }

    Eigen::VectorXd xh = solver.solve(x0);
    Eigen::VectorXd yh = solver.solve(y0);
    Eigen::VectorXd zh = solver.solve(z0);

    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Failed to solve fairing system");
    }

    update_mesh_from_solution(obj.mesh, xh, yh, zh);
    compute_vertex_normals(obj.mesh);
}

void build_drawables() {
    g_drawables.clear();
    g_drawables.reserve(g_render_objects.size());

    for (const auto& src : g_render_objects) {
        DrawableObject drawable;
        drawable.vertices.reserve(src.mesh.obj.faces.size() * 9);
        drawable.normals.reserve(src.mesh.obj.faces.size() * 9);

        std::copy(std::begin(src.ambient), std::end(src.ambient), drawable.ambient);
        std::copy(std::begin(src.diffuse), std::end(src.diffuse), drawable.diffuse);
        std::copy(std::begin(src.specular), std::end(src.specular), drawable.specular);
        drawable.shininess = src.shininess;

        for (const auto& face : src.mesh.obj.faces) {
            const Vertex& v1 = src.mesh.obj.vertices.at(face.idx1);
            const Vertex& v2 = src.mesh.obj.vertices.at(face.idx2);
            const Vertex& v3 = src.mesh.obj.vertices.at(face.idx3);

            const Vec3f& n1 = src.mesh.vertex_normals.at(face.idx1);
            const Vec3f& n2 = src.mesh.vertex_normals.at(face.idx2);
            const Vec3f& n3 = src.mesh.vertex_normals.at(face.idx3);

            const std::array<const Vertex*, 3> verts{&v1, &v2, &v3};
            const std::array<const Vec3f*, 3> norms{&n1, &n2, &n3};

            for (int i = 0; i < 3; ++i) {
                drawable.normals.push_back(norms[i]->x);
                drawable.normals.push_back(norms[i]->y);
                drawable.normals.push_back(norms[i]->z);

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

    GLfloat color[4] = {1.f, 1.f, 1.f, 1.f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, color);
}

void init_gl() {
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(g_scene.cam_transforms.P.data());

    glMatrixMode(GL_MODELVIEW);
    init_lights();
}

void set_lights() {
    const auto& lights = g_scene.lights;
    int num_lights = static_cast<int>(lights.size());

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

static double camera_aspect_from_P(const Eigen::Matrix4d& P) {
    return P(1,1) / P(0,0);
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

void run_fairing() {
    for (auto& obj : g_render_objects) {
        apply_implicit_fairing(obj, g_time_step);
    }
    build_drawables();
    glutPostRedisplay();
}

void keyboard(unsigned char key, int, int) {
    if (key == 27 || key == 'q' || key == 'Q') {
        std::exit(0);
    } else if (key == 'f' || key == 'F') {
        run_fairing();
    }
}

RenderObject make_render_object(const ObjectInstance& inst) {
    RenderObject obj;
    obj.mesh.obj = inst.obj;

    obj.mesh.vertex_ptrs.reserve(obj.mesh.obj.vertices.size());
    for (std::size_t i = 0; i < obj.mesh.obj.vertices.size(); ++i) {
        obj.mesh.vertex_ptrs.push_back(&obj.mesh.obj.vertices[i]);
    }

    obj.mesh.face_ptrs.reserve(obj.mesh.obj.faces.size());
    for (std::size_t i = 0; i < obj.mesh.obj.faces.size(); ++i) {
        obj.mesh.face_ptrs.push_back(&obj.mesh.obj.faces[i]);
    }

    Mesh_Data mesh_data;
    mesh_data.vertices = &obj.mesh.vertex_ptrs;
    mesh_data.faces = &obj.mesh.face_ptrs;

    if (!build_HE(&mesh_data, &obj.mesh.hevs, &obj.mesh.hefs)) {
        throw std::runtime_error("Failed to build halfedge structure");
    }

    for (std::size_t i = 1; i < obj.mesh.hevs.size(); ++i) {
        obj.mesh.hevs[i]->index = static_cast<int>(i - 1);
    }

    auto fill_material = [](const Eigen::Vector3d& src, GLfloat out[4]) {
        out[0] = static_cast<GLfloat>(src[0]);
        out[1] = static_cast<GLfloat>(src[1]);
        out[2] = static_cast<GLfloat>(src[2]);
        out[3] = 1.0f;
    };

    fill_material(inst.ambient, obj.ambient);
    fill_material(inst.diffuse, obj.diffuse);
    fill_material(inst.specular, obj.specular);
    double shininess = std::max(0.0, std::min(inst.shininess, 128.0));
    obj.shininess = static_cast<GLfloat>(shininess);

    compute_vertex_normals(obj.mesh);
    return obj;
}

void build_render_objects() {
    g_render_objects.clear();
    g_render_objects.reserve(g_scene.scene_objects.size());
    for (const auto& inst : g_scene.scene_objects) {
        g_render_objects.push_back(make_render_object(inst));
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres] [h]\n";
        return 1;
    }

    const std::size_t xres = parse_size_t(argv[2]);
    const std::size_t yres = parse_size_t(argv[3]);
    g_time_step = std::stod(argv[4]);

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

    build_render_objects();
    build_drawables();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_window_width, g_window_height);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Implicit Fairing");

#ifdef __APPLE__
    glewExperimental = GL_TRUE;
#endif
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

