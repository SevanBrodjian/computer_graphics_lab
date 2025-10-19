#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include "scene_types.h"
#include "io_utils.h"
#include <Eigen/Dense>

using Eigen::Matrix4d;


Matrix4d make_translation(double tx, double ty, double tz);
Matrix4d make_scaling(double sx, double sy, double sz);
Matrix4d make_rotation(double rx, double ry, double rz, double angle);

void apply_transform_to_object(Object& src, const Matrix4d& M, bool transform_normals=true);

Camera make_cam_matrices(const CameraParams& cam);

void world_to_view(Scene& scene);

void view_to_ndc(Scene& scene);
void view_to_ndc(std::vector<Vertex>& verts, const Scene& scene);

void ndc_to_screen(const Image& img, Scene& scene);
void ndc_to_screen(const Image& img, std::vector<Vertex>& verts);

#endif