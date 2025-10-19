#ifndef SHADING_UTILS_H
#define SHADING_UTILS_H

#include "scene_types.h"
#include <Eigen/Dense>

using Eigen::Vector3d;


Vector3d lighting (const Vector3d& P, const Vector3d& n_in, const ObjectInstance& mat, const std::vector<Light>& lights, Vector3d e = Vector3d::Zero());
void shade_by_mode(Image& img, Scene& scenes, size_t mode); 
void draw_wireframe(Image& img, Scene& scene);

#endif