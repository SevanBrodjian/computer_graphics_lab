#ifndef HW3_SCENE_LOADER_H
#define HW3_SCENE_LOADER_H

#include "scene_types.h"

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

// Reused from hw2

std::string parse_parent_path(const std::string& path);
std::size_t parse_size_t(const char* str);
Scene parse_scene_file(std::ifstream& fin, const std::string& parent_path);

#endif
