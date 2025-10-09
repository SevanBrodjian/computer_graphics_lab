#include "transform_utils.h"
#include "io_utils.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;


void object::print() const {
        std::cout << filename << ":" << std::endl << std::endl;

        for (std::size_t i = 1; i < vertices.size(); ++i) std::cout << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << std::endl;
        for (const auto &f : faces) std::cout << "f " << f.v1 << " " << f.v2 << " " << f.v3 << std::endl;
}

Matrix4d makeTranslation(double tx, double ty, double tz) {
    Matrix4d T = Matrix4d::Identity();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    return T;
}

Matrix4d makeScaling(double sx, double sy, double sz) {
    Matrix4d S = Matrix4d::Identity();
    S(0, 0) = sx;
    S(1, 1) = sy;
    S(2, 2) = sz;
    return S;
}

Matrix4d makeRotation(double rx, double ry, double rz, double angle) {
    Vector3d axis(rx, ry, rz);
    if (axis.norm() == 0.0) { // Zero axis
        return Matrix4d::Identity();
    }
    axis.normalize();
    Matrix3d R3 = AngleAxisd(angle, axis).toRotationMatrix(); // 3x3 rotation matrix (not homogeneous)
    Matrix4d R = Matrix4d::Identity();
    R.block<3,3>(0,0) = R3; // Set upper-left 3x3 block to the calculated rotation matrix
    return R;
}

object applyTransformToObject(const object& src, const Matrix4d& M) {
    object out = src;
    for (size_t vi = 1; vi < out.vertices.size(); ++vi) {
        auto& v = out.vertices[vi];
        Eigen::Vector4d p(v.x, v.y, v.z, 1.0);
        Eigen::Vector3d q = (M * p).hnormalized();
        v.x = q[0];
        v.y = q[1];
        v.z = q[2];
    }
    return out;
}

CameraTransforms makeCameraMatrices(const CameraParams& cam){
    // Make transformation matrix
    Matrix4d T_C = makeTranslation(cam.px, cam.py, cam.pz);
    Matrix4d R_C = makeRotation(cam.ox, cam.oy, cam.oz, cam.oang);
    Matrix4d C_inv = (T_C * R_C).inverse();

    // Make perspective matrix
    double n = cam.znear;
    double f = cam.zfar;
    double l = cam.left;
    double r = cam.right;
    double b = cam.bottom;
    double t = cam.top;

    Eigen::Matrix4d P;
    P << 
        (2 * n) / (r - l),  0,                  (r + l) / (r - l),   0,
        0,                  (2 * n) / (t - b),  (t + b) / (t - b),   0,
        0,                  0,                  -(f + n) / (f - n),  -(2 * f * n) / (f - n),
        0,                  0,                  -1,                  0;

    return CameraTransforms({C_inv, P});
}

std::vector<object> applyCameraTransformsToObjects(const std::vector<object>& objects, CameraTransforms cam_transforms){
    std::vector<object> ndc_objects;
    Matrix4d cameraTransformMatrix = cam_transforms.P * cam_transforms.Cinv;
    for (const auto& obj : objects) {
        ndc_objects.push_back(applyTransformToObject(obj, cameraTransformMatrix));
    }
    return ndc_objects;
}

std::vector<vertex> convertCoordsToPixels(std::vector<object> objects, size_t xres, size_t yres){
    std::vector<vertex> pixelCoords;
    for (const auto& obj : objects) {
        for (const auto& vert : obj.vertices) {
            if (vert.x < -1 || vert.x > 1 || vert.y < -1 || vert.y > 1 || vert.z < -1 || vert.z > 1) continue;

            double pixelX = (vert.x + 1) / 2 * xres;
            double pixelY = (vert.y + 1) / 2 * yres;
            pixelCoords.push_back(vertex({pixelX, pixelY, vert.z}));
        }
    }
    return pixelCoords;
}