#include "io_utils.h"
#include "transform_utils.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <optional>
#include <stdexcept>
#include <cctype>

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;


std::string join_path(const std::string& parent, const std::string& filename) {
    if (parent.empty()) return filename;
    if (parent.back() == '/' || parent.back() == '\\') return parent + filename;
    return parent + "/" + filename;
}

std::vector<object> loadObjects(const std::vector<std::string>& fpaths, std::string parent_path) {
    std::vector<object> objects;

    for (const auto& filename : fpaths) {
        std::string file_path = join_path(parent_path, filename);
        std::ifstream file(file_path);
        if (!file) {
            std::cerr << "Error: Could not open file " << file_path << std::endl;
            continue;
        }

        std::vector<vertex> vertices{{0.0, 0.0, 0.0}};
        std::vector<face> faces;
        std::string line;

        while (std::getline(file, line)) {
            std::string s = line;
            auto p = s.find_first_not_of(" \t\r\n");
            if (p == std::string::npos) continue;
            if (s[p] == '#') continue;

            char type = s[p];
            std::istringstream iss(s.substr(p + 1));

            if (type != 'v' && type != 'f') {
                throw std::runtime_error("Invalid format: must start with 'v' or 'f'");
            }

            if (type == 'v') {
                double x, y, z;
                if (!(iss >> x >> y >> z)) throw std::runtime_error("Invalid vertex format");
                std::string leftover;
                if (iss >> leftover) throw std::runtime_error("Extra data in vertex");
                vertices.push_back({x, y, z});
            } else {
                unsigned int a, b, c;
                if (!(iss >> a >> b >> c)) throw std::runtime_error("Invalid face format");
                std::string leftover;
                if (iss >> leftover) throw std::runtime_error("Extra data in face");
                if (a >= vertices.size() || b >= vertices.size() || c >= vertices.size()) {
                    throw std::runtime_error("Face index out of range");
                }
                faces.push_back({a, b, c});
            }
        }

        objects.push_back({file_path, vertices, faces});
    }

    return objects;
}

Matrix4d makeTransformFromLines(const std::vector<std::string>& lines) {
    Matrix4d M = Matrix4d::Identity();
    size_t lineno = 0;

    for (const auto& raw_line : lines) {
        ++lineno;

        // Skip empty lines and comments
        auto first_non_ws = raw_line.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (raw_line[first_non_ws] == '#') continue;

        std::istringstream iss(raw_line.substr(first_non_ws));
        char kind;
        iss >> kind;
        if (!iss) continue;

        Matrix4d T = Matrix4d::Identity();
        if (kind == 't') {
            double tx, ty, tz;
            if (!(iss >> tx >> ty >> tz)) {
                throw std::runtime_error("Invalid translation at line " + std::to_string(lineno));
            }
            T = makeTranslation(tx, ty, tz);
        } else if (kind == 's') {
            double sx, sy, sz;
            if (!(iss >> sx >> sy >> sz)) {
                throw std::runtime_error("Invalid scale at line " + std::to_string(lineno));
            }
            T = makeScaling(sx, sy, sz);
        } else if (kind == 'r') {
            double rx, ry, rz, angle;
            if (!(iss >> rx >> ry >> rz >> angle)) {
                throw std::runtime_error("Invalid rotation at line " + std::to_string(lineno));
            }
            T = makeRotation(rx, ry, rz, angle);
        } else {
            std::cerr << "Warning: unknown transform type '" << kind << "' on line " << lineno << std::endl;
            continue;
        }

        // Left-multiply
        M = T * M;
    }

    return M;
};

size_t find_string_idx(
    const std::string& name, const std::unordered_map<std::string, 
    size_t>& name_to_idx
) {
    auto it = name_to_idx.find(name);
    if (it == name_to_idx.end()) {
        throw std::out_of_range("Name not found: " + name);
    }
    return it->second;
}

std::size_t parseObjectMappings(
    const std::vector<std::string>& lines,
    std::vector<std::string>& object_names,
    std::vector<std::string>& object_paths
) {
    bool started_mapping = false;
    std::size_t i = 0;

    for (; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        auto first_non_ws = line.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) {
            if (started_mapping) { ++i; break; }
            else continue;
        }
        if (line[first_non_ws] == '#') continue;

        started_mapping = true;

        std::istringstream iss(line.substr(first_non_ws));
        std::string name, path;
        if (!(iss >> name >> path)) {
            std::cerr << "Warning: Cannot read line: " << line << std::endl;
            continue;
        }
        object_names.push_back(name);
        object_paths.push_back(path);
    }
    return i; // index of first line after the mapping section (roughly where transforms begin)
}

void processTransformBlocks(
    const std::vector<std::string>& lines,
    std::size_t start_idx,
    const std::vector<object>& objects,
    const std::vector<std::string>& object_names,
    const std::unordered_map<std::string, std::size_t>& name_to_idx,
    std::vector<object>& out_transformed,
    std::vector<std::string>& out_names
) {
    if (objects.size() != object_names.size()) {
        throw std::runtime_error("Loaded different number of objects and names.");
    }

    std::unordered_map<std::string, std::size_t> copy_count;

    std::string current_name;
    std::vector<std::string> current_transform_lines;

    auto flush_block = [&]() {
        if (current_name.empty()) {
            current_transform_lines.clear();
            return;
        }
        try {
            size_t base_idx = find_string_idx(current_name, name_to_idx);
            Eigen::Matrix4d M = makeTransformFromLines(current_transform_lines);
            object transformed = applyTransformToObject(objects.at(base_idx), M);
            std::size_t n = ++copy_count[current_name];
            out_names.push_back(current_name + "_copy" + std::to_string(n));
            out_transformed.push_back(std::move(transformed));
        } catch (const std::exception& e) {
            std::cerr << "Error processing block for '" << current_name << "': " << e.what() << std::endl;
        }
        current_transform_lines.clear();
        current_name.clear();
    };

    for (std::size_t i = start_idx; i < lines.size(); ++i) {
        const std::string& tline = lines[i];
        auto first_non_ws = tline.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) { flush_block(); continue; } // Empty line separating objects
        if (tline[first_non_ws] == '#') continue;

        // We are still in the transforms for this object -> add the line and continue
        std::istringstream iss(tline.substr(first_non_ws));
        std::string tok;
        iss >> tok;
        if (!iss && tok.empty()) continue;

        // transform codes start with t/r/s
        if (tok == "t" || tok == "r" || tok == "s") {
            if (current_name.empty()) {
                std::cerr << "Warning: Transform before object name, skipping line " << tline << std::endl;
                continue;
            }
            current_transform_lines.push_back(tline.substr(first_non_ws));
        } else {
            // new object block begins (not necessarily whitespace separated)
            flush_block();
            current_name = tok;
        }
    }
    flush_block();
}

TransformRunResult makeTransformedObjectsFromLines(const std::vector<std::string>& lines, std::string parent_path) {
    // Parse mapping
    std::vector<std::string> object_names;
    std::vector<std::string> object_paths;
    std::size_t next_idx = parseObjectMappings(lines, object_names, object_paths);

    // Load base objects in the same order
    std::vector<object> objects = loadObjects(object_paths, parent_path);

    // Build name to index mapping
    std::unordered_map<std::string, std::size_t> name_to_idx;
    name_to_idx.reserve(object_names.size());
    for (std::size_t i = 0; i < object_names.size(); ++i) {
        name_to_idx.emplace(object_names[i], i);
    }

    // Process transform blocks
    TransformRunResult result;
    processTransformBlocks(
        lines,
        next_idx,
        objects,
        object_names,
        name_to_idx,
        result.transformed_objects,
        result.transformed_object_names
    );

    return result;
}

void readCameraParams(CameraParams& cam, std::ifstream& fin){
    std::string raw;
    bool in_objects = false;

    while (std::getline(fin, raw)) {
        auto first_non_ws = raw.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (raw[first_non_ws] == '#') continue;
        std::string line = raw.substr(first_non_ws);

        if (line == "objects:") { in_objects = true; break; }

        // parse "key ..." lines
        std::istringstream iss(line);
        std::string key; iss >> key;

        if (key == "position") {
            iss >> cam.px >> cam.py >> cam.pz;
            if (!iss) std::cerr << "Warning: 'position' expects 3 numbers.\n";
        } else if (key == "orientation") {
            iss >> cam.ox >> cam.oy >> cam.oz >> cam.oang;
            if (!iss) std::cerr << "Warning: 'orientation' expects 4 numbers.\n";
        } else if (key == "near") {
            iss >> cam.znear;
        } else if (key == "far") {
            iss >> cam.zfar;
        } else if (key == "left") {
            iss >> cam.left;
        } else if (key == "right") {
            iss >> cam.right;
        } else if (key == "top") {
            iss >> cam.top;
        } else if (key == "bottom") {
            iss >> cam.bottom;
        } else {
            std::cerr << "Note: ignoring unknown camera key '" << key << "'.\n";
        }
    }

    if (cam.znear==0 || cam.zfar==cam.znear ||
        cam.right==cam.left || cam.top==cam.bottom) {
        throw std::runtime_error("Invalid frustum parameters");
    }

    if (!in_objects) {
        std::cerr << "Missing 'objects:' after camera section.\n";
    }
}

ParseSceneFileResult parseSceneFile(std::ifstream& fin, std::string parent_path){
    CameraParams cam;
    std::vector<std::string> object_section_lines;

    std::string raw;
    bool in_camera = false;

    // 1) Find "camera:" first (ignore preamble/comments/blank lines)
    while (std::getline(fin, raw)) {
        // Ignore blank lines and comments
        auto first_non_ws = raw.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (raw[first_non_ws] == '#') continue;
        std::string line = raw.substr(first_non_ws);
        if (line == "camera:") { in_camera = true; break; }
    }
    if (!in_camera) {std::cerr << "Missing 'camera:' section.\n";}

    // 2) Read camera fields until we see "objects:"
    readCameraParams(cam, fin);

    // 3) Collect everything under "objects:" as lines (left-trimmed)
    while (std::getline(fin, raw)) {
        object_section_lines.push_back(raw);
    }

    // 4) Get transformed objects from objects: section
    TransformRunResult scene_objects = makeTransformedObjectsFromLines(object_section_lines, parent_path);

    return ParseSceneFileResult({cam, scene_objects});
}

void writePPM(const std::vector<uint8_t>& img, size_t xres, size_t yres){
    std::cout << "P3\n" << xres << " " << yres << "\n255\n";
    for (size_t i = 0; i < xres * yres * 3; i += 3) {
        std::cout << static_cast<int>(img[i]) << " "
                << static_cast<int>(img[i+1]) << " "
                << static_cast<int>(img[i+2]) << "\n";
    }
}