#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "scene_types.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <cstddef>
#include <fstream>
#include <Eigen/Dense>

std::string join_path(const std::string& parent, const std::string& filename);

std::vector<object> loadObjects(const std::vector<std::string>& fpaths,
                                std::string parent_path);

Eigen::Matrix4d makeTransformFromLines(const std::vector<std::string>& lines);

size_t find_string_idx(const std::string& name,
    const std::unordered_map<std::string, size_t>& name_to_idx);

std::size_t parseObjectMappings(const std::vector<std::string>& lines,
    std::vector<std::string>& object_names,
    std::vector<std::string>& object_paths);

void processTransformBlocks(const std::vector<std::string>& lines,
    std::size_t start_idx,
    const std::vector<object>& objects,
    const std::vector<std::string>& object_names,
    const std::unordered_map<std::string, std::size_t>& name_to_idx,
    std::vector<object>& out_transformed,
    std::vector<std::string>& out_names);

struct TransformRunResult {
    std::vector<object> transformed_objects;
    std::vector<std::string> transformed_object_names;
};

TransformRunResult makeTransformedObjectsFromLines(const std::vector<std::string>& lines,
                                                   std::string parent_path);

struct ParseSceneFileResult {
    CameraParams camera_params;
    TransformRunResult scene_objects;
};

ParseSceneFileResult parseSceneFile(std::ifstream& fin, std::string parent_path);

void writePPM(const std::vector<uint8_t>& img, size_t xres, size_t yres);

#endif