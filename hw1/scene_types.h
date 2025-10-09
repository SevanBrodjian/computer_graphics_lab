#ifndef SCENE_TYPES_H
#define SCENE_TYPES_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <cstdint>

struct vertex {
    double x, y, z;
};

struct face {
    unsigned int v1, v2, v3;
};

struct object {
    std::string filename;
    std::vector<vertex> vertices;
    std::vector<face> faces;
    void print() const;
};

struct CameraParams {
    double px = 0, py = 0, pz = 0;            // position
    double ox = 0, oy = 1, oz = 0, oang = 0;  // orientation axis + angle (radians)
    double znear = 0, zfar = 0;
    double left = 0, right = 0, top = 0, bottom = 0;
};

struct CameraTransforms {
    Eigen::Matrix4d Cinv;  // world→camera inverse extrinsic
    Eigen::Matrix4d P;     // projection matrix
};

struct color {
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