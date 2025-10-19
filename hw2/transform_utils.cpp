#include "io_utils.h"
#include "transform_utils.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;


Matrix4d make_translation(double tx, double ty, double tz) {
    Matrix4d T = Matrix4d::Identity();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    return T;
}

Matrix4d make_scaling(double sx, double sy, double sz) {
    Matrix4d S = Matrix4d::Identity();
    S(0, 0) = sx;
    S(1, 1) = sy;
    S(2, 2) = sz;
    return S;
}

Matrix4d make_rotation(double rx, double ry, double rz, double angle) {
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

void apply_transform_to_object(Object& src, const Matrix4d& M, bool transform_normals) {
    // Transform vertices
    for (size_t vi = 1; vi < src.vertices.size(); ++vi) {
        auto& v = src.vertices[vi];
        Eigen::Vector4d p(v.x, v.y, v.z, 1.0);
        Eigen::Vector3d q = (M * p).hnormalized();
        v.x = q[0];
        v.y = q[1];
        v.z = q[2];
    }

    // Transform normals
    if (transform_normals) {
        // Ignore translations, take the upper-left 3x3 block
        const Eigen::Matrix3d A = M.block<3,3>(0,0);
        Eigen::Matrix3d N;
        double det = A.determinant();
        if (std::abs(det) < 1e-15) {
            N = Eigen::Matrix3d::Identity();
            std::cerr << "Warning: singular transform for normals. Using identity.\n";
        } else {
            N = A.inverse().transpose();
        }
        
        for (size_t ni = 1; ni < src.normals.size(); ++ni) {
            auto& vn = src.normals[ni];
            Eigen::Vector3d n(vn.x, vn.y, vn.z);
            n = N * n;
            n.normalize();
            vn.x = n.x();
            vn.y = n.y();
            vn.z = n.z();
        }
    }
}

Camera make_cam_matrices(const CameraParams& cam){
    // Make transformation matrix
    Matrix4d T_C = make_translation(cam.px, cam.py, cam.pz);
    Matrix4d R_C = make_rotation(cam.ox, cam.oy, cam.oz, cam.oang);
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

    return Camera({C_inv, P});
}

void world_to_view(Scene& scene) {
    for (auto& obj_inst : scene.scene_objects) {
        apply_transform_to_object(obj_inst.obj, scene.cam_transforms.Cinv, true);
    }
    
    // Transform lights
    for (auto& lt : scene.lights) {
        Eigen::Vector4d p(lt.x, lt.y, lt.z, 1.0);
        Eigen::Vector3d q = (scene.cam_transforms.Cinv * p).hnormalized();
        lt.x = q[0];
        lt.y = q[1];
        lt.z = q[2];
    }
}

void view_to_ndc(Scene& scene) {
    for (auto& obj_inst : scene.scene_objects) {
        // Don't convert normals to NDC
        apply_transform_to_object(obj_inst.obj, scene.cam_transforms.P, false);
    }
}

void view_to_ndc(std::vector<Vertex>& verts, const Scene& scene) {
    for (auto& v : verts) {
        Eigen::Vector4d p(v.x, v.y, v.z, 1.0);
        Eigen::Vector3d q = (scene.cam_transforms.P * p).hnormalized();
        v.x = q[0];
        v.y = q[1];
        v.z = q[2];
    }
}

void ndc_to_screen(const Image& img, Scene& scene){
    if (img.xres == 0 || img.yres == 0) return;

    const double max_x = static_cast<double>(img.xres - 1);
    const double max_y = static_cast<double>(img.yres - 1);

    for (auto& obj_inst : scene.scene_objects) {
        for (std::size_t vi = 1; vi < obj_inst.obj.vertices.size(); ++vi) {
            auto& vert = obj_inst.obj.vertices[vi];

            vert.x = (vert.x + 1.0) * 0.5 * max_x;
            vert.y = (vert.y + 1.0) * 0.5 * max_y;
        }
    }
}

void ndc_to_screen(const Image& img, std::vector<Vertex>& verts) {
    if (img.xres == 0 || img.yres == 0) return;

    const double max_x = static_cast<double>(img.xres - 1);
    const double max_y = static_cast<double>(img.yres - 1);

    for (auto& v : verts) {
        v.x = (v.x + 1.0) * 0.5 * max_x;
        v.y = (v.y + 1.0) * 0.5 * max_y;
    }
}