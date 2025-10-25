#ifndef HW3_SCENE_TYPES_H
#define HW3_SCENE_TYPES_H

#include <string>
#include <vector>
#include <cstdint>

#include "linalg.h"

struct Vertex {
    double x, y, z;
};

struct Normal {
    double x, y, z;
};

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
    Vec3 ambient;
    Vec3 diffuse;
    Vec3 specular;
    double shininess;
};

struct Light {
    double x, y, z;
    double r, g, b;
    double atten;
};

struct CameraParams {
    double px = 0, py = 0, pz = 0;
    double ox = 0, oy = 1, oz = 0, oang = 0;
    double znear = 0, zfar = 0;
    double left = 0, right = 0, top = 0, bottom = 0;
};

struct Camera {
    Mat4 Cinv;
    Mat4 P;
};

struct Scene {
    Camera cam_transforms;
    std::vector<ObjectInstance> scene_objects;
    std::vector<Light> lights;
};

#endif
