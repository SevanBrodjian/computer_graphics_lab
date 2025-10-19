#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "scene_types.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <cstddef>
#include <fstream>
#include <Eigen/Dense>


std::string parse_parent_path(const std::string& path);

size_t parse_size_t(const char* str);

std::string join_path(const std::string& parent, const std::string& filename);

std::vector<Object> load_objects(const std::vector<std::string>& fpaths,
                                std::string parent_path);

Eigen::Matrix4d make_transform_from_lines(const std::vector<std::string>& lines);

size_t find_string_idx(const std::string& name,
    const std::unordered_map<std::string, size_t>& name_to_idx);

std::size_t parse_object_mappings(const std::vector<std::string>& lines,
    std::vector<std::string>& object_names,
    std::vector<std::string>& object_paths);

void process_transform_blocks(const std::vector<std::string>& lines,
    std::size_t start_idx,
    const std::vector<Object>& objects,
    const std::vector<std::string>& object_names,
    const std::unordered_map<std::string, std::size_t>& name_to_idx,
    std::vector<ObjectInstance>& out_transformed);

std::vector<ObjectInstance> make_transformed_objects_from_lines(const std::vector<std::string>& lines,
                                                   std::string parent_path);

Scene parse_scene_file(std::ifstream& fin, std::string parent_path);

void write_ppm(const Image& img);

#endif