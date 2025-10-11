#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include "scene_types.h"
#include <Eigen/Dense>

Eigen::Matrix4d makeTranslation(double tx, double ty, double tz);
Eigen::Matrix4d makeScaling(double sx, double sy, double sz);
Eigen::Matrix4d makeRotation(double rx, double ry, double rz, double angle);

object applyTransformToObject(const object& src, const Eigen::Matrix4d& M);

CameraTransforms makeCameraMatrices(const CameraParams& cam);

std::vector<object> applyCameraTransformsToObjects(const std::vector<object>& objects, CameraTransforms cam_transforms);

void convertCoordsToPixels(std::vector<object>& objects, size_t xres, size_t yres);

#endif