#ifndef HW3_SCENE_TYPES_H
#define HW3_SCENE_TYPES_H

#include <Eigen/Dense>
#include <string>
#include <vector>

// Reused from hw2/hw3 but simplified for meshes without normals

struct Vertex {
    double x, y, z;
};

struct Face {
    unsigned int v1, v2, v3;
};

struct Object {
    std::string filename;
    // 1-indexed vertex list, vertices[0] is dummy to align with OBJ indices
    std::vector<Vertex> vertices;
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

struct CameraParams {
    double px = 0, py = 0, pz = 0;
    double ox = 0, oy = 1, oz = 0, oang = 0;
    double znear = 0, zfar = 0;
    double left = 0, right = 0, top = 0, bottom = 0;
};

struct Camera {
    Eigen::Matrix4d Cinv;
    Eigen::Matrix4d P;
};

struct Scene {
    Camera cam_transforms;
    std::vector<ObjectInstance> scene_objects;
    std::vector<Light> lights;
};

#endif
