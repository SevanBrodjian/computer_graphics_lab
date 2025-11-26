/* CS/CNS 171 -- HW6 Part 2
 * Keyframe animation of the I-Bar using Catmull-Rom interpolation.
 */
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <GL/glew.h>
#include <GL/glut.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Keyframe {
    int frame = 0;
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    Eigen::Vector3f scale = Eigen::Vector3f::Ones();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
};

std::vector<Keyframe> keyframes;
int totalFrames = 0;
int currentFrame = 0;
GLUquadricObj *quadratic = nullptr;

// Catmull-Rom spline interpolation for vectors/quaternions.
template <typename T>
T catmullRom(const T &p0, const T &p1, const T &p2, const T &p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return 0.5f * ((2.f * p1) + (-p0 + p2) * t +
                   (2.f * p0 - 5.f * p1 + 4.f * p2 - p3) * t2 +
                   (-p0 + 3.f * p1 - 3.f * p2 + p3) * t3);
}

Keyframe interpolateFrame(int frameIdx) {
    if (keyframes.empty()) {
        return Keyframe();
    }

    // Ensure keyframes are sorted by frame index.
    std::sort(keyframes.begin(), keyframes.end(),
              [](const Keyframe &a, const Keyframe &b) { return a.frame < b.frame; });

    int n = static_cast<int>(keyframes.size());
    int idx1 = 0;
    for (int i = 0; i < n; ++i) {
        if (frameIdx >= keyframes[i].frame) {
            idx1 = i;
        } else {
            break;
        }
    }

    int idx2 = (idx1 + 1) % n;
    const Keyframe &k1 = keyframes[idx1];
    const Keyframe &k2 = keyframes[idx2];

    float f1 = static_cast<float>(k1.frame);
    float f2 = static_cast<float>(k2.frame);
    float frameValue = static_cast<float>(frameIdx);

    // Handle looping from last keyframe back to the first.
    if (idx2 == 0) {
        f2 += static_cast<float>(totalFrames);
        if (frameIdx < k1.frame) {
            frameValue += static_cast<float>(totalFrames);
        }
    }

    float t = (frameValue - f1) / (f2 - f1);

    const Keyframe &k0 = keyframes[(idx1 - 1 + n) % n];
    const Keyframe &k3 = keyframes[(idx2 + 1) % n];

    Keyframe result;
    result.translation = catmullRom(k0.translation, k1.translation, k2.translation, k3.translation, t);
    result.scale = catmullRom(k0.scale, k1.scale, k2.scale, k3.scale, t);

    Eigen::Vector4f q0 = k0.rotation.coeffs();
    Eigen::Vector4f q1 = k1.rotation.coeffs();
    Eigen::Vector4f q2 = k2.rotation.coeffs();
    Eigen::Vector4f q3 = k3.rotation.coeffs();
    Eigen::Vector4f qInterp = catmullRom(q0, q1, q2, q3, t);
    Eigen::Quaternionf q;
    q.coeffs() = qInterp;
    q.normalize();
    result.rotation = q;

    result.frame = frameIdx;
    return result;
}

bool loadScript(const std::string &filename) {
    std::ifstream infile(filename.c_str());
    if (!infile) {
        std::cerr << "Failed to open script file: " << filename << std::endl;
        return false;
    }

    keyframes.clear();
    infile >> totalFrames;
    std::string token;
    while (infile >> token) {
        if (token != "Frame") {
            break;
        }
        Keyframe kf;
        infile >> kf.frame;

        std::string label;
        infile >> label >> kf.translation[0] >> kf.translation[1] >> kf.translation[2];
        infile >> label >> kf.scale[0] >> kf.scale[1] >> kf.scale[2];

        float axisX, axisY, axisZ, angleDeg;
        infile >> label >> axisX >> axisY >> axisZ >> angleDeg;
        float angleRad = angleDeg * static_cast<float>(M_PI) / 180.0f;
        Eigen::Vector3f axis(axisX, axisY, axisZ);
        if (axis.norm() < 1e-6f) {
            axis = Eigen::Vector3f::UnitX();
        } else {
            axis.normalize();
        }
        kf.rotation = Eigen::AngleAxisf(angleRad, axis);

        keyframes.push_back(kf);
    }

    std::sort(keyframes.begin(), keyframes.end(),
              [](const Keyframe &a, const Keyframe &b) { return a.frame < b.frame; });

    return !keyframes.empty();
}

void drawIBar() {
    float cyRad = 0.2f, cyHeight = 1.0f;
    int quadStacks = 4, quadSlices = 4;

    glPushMatrix();
    glColor3f(0, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 1, 0, 0);
    gluCylinder(quadratic, cyRad, cyRad, 2.0f * cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0, 1, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(1, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(1, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 40.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    Keyframe kf = interpolateFrame(currentFrame);

    glTranslatef(kf.translation[0], kf.translation[1], kf.translation[2]);
    Eigen::AngleAxisf aa(kf.rotation);
    float angleDeg = aa.angle() * 180.0f / static_cast<float>(M_PI);
    Eigen::Vector3f axis = aa.axis();
    glRotatef(angleDeg, axis[0], axis[1], axis[2]);
    glScalef(kf.scale[0], kf.scale[1], kf.scale[2]);

    drawIBar();

    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 60.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int) {
    // Step forward one frame on any key press.
    currentFrame = (currentFrame + 1) % totalFrames;
    glutPostRedisplay();
}

void initGL() {
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    quadratic = gluNewQuadric();
}

int main(int argc, char **argv) {
    std::string scriptFile = "test.script";
    if (argc > 1) {
        scriptFile = argv[1];
    }

    if (!loadScript(scriptFile)) {
        return 1;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("I-Bar Keyframe");

#ifndef __APPLE__
    glewInit();
#endif

    initGL();

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);

    glutMainLoop();
    return 0;
}
