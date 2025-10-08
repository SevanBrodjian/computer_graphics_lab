#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include "io_utils.h"

#include <vector>
#include <string>
#include <Eigen/Dense>

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

Eigen::Matrix4d makeTranslation(double tx, double ty, double tz);

Eigen::Matrix4d makeScaling(double sx, double sy, double sz);

Eigen::Matrix4d makeRotation(double rx, double ry, double rz, double angle);

object applyTransformToObject(const object& src, const Eigen::Matrix4d& M);

struct CameraTransforms {
    Eigen::Matrix4d Cinv;
    Eigen::Matrix4d P;
};

CameraTransforms makeCameraMatrices(CameraParams cam);

#endif