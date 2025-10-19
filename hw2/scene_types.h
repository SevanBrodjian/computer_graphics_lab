#ifndef SCENE_TYPES_H
#define SCENE_TYPES_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <cstdint>

using Eigen::Vector3d;
using Eigen::Map;


struct Vertex {
    double x, y, z;
};

struct Normal {
    double x, y, z;
};

// Maps to convert vertices and normals to Vector3d, useful for math, with const handling.
inline Map<const Vector3d> as_vec3(const Vertex& v) { return Map<const Vector3d>(&v.x); }
inline Map<Vector3d>       as_vec3(      Vertex& v) { return Map<Vector3d>(&v.x); }
inline Map<const Vector3d> as_vec3(const Normal& n) { return Map<const Vector3d>(&n.x); }
inline Map<Vector3d>       as_vec3(      Normal& n) { return Map<Vector3d>(&n.x); }

struct Face {
    unsigned int v1, v2, v3;
    unsigned int vn1, vn2, vn3;
};

struct Object {
    std::string filename;
    std::vector<Vertex> vertices;
    std::vector<Normal> normals;
    std::vector<Face> faces;
};

struct ObjectInstance {
    Object obj;
    std::string name;
    Eigen::Vector3d ambient;
    Eigen::Vector3d diffuse;
    Eigen::Vector3d specular;
    double shininess;
};

struct Light {
    double x, y, z;
    double r, g, b;
    double atten;
};

struct Image {
    std::vector<uint8_t> img;
    std::vector<double> z_buf;
    size_t xres;
    size_t yres;
};

struct CameraParams {
    double px = 0, py = 0, pz = 0; // position
    double ox = 0, oy = 1, oz = 0, oang = 0;  // orientation axis + angle (radians)
    double znear = 0, zfar = 0;
    double left = 0, right = 0, top = 0, bottom = 0;
};

struct Camera {
    Eigen::Matrix4d Cinv; // world to camera
    Eigen::Matrix4d P; // projection matrix
};

struct Scene {
    Camera cam_transforms;
    std::vector<ObjectInstance> scene_objects;
    std::vector<Light> lights;
};

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    void print() {
        std::cout << static_cast<int>(r) << " "
                << static_cast<int>(g) << " "
                << static_cast<int>(b) << "\n";
    }
};

#endif