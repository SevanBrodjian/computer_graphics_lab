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


std::string parse_parent_path(const std::string& path) {
    std::size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return "";
    return path.substr(0, pos);
}

size_t parse_size_t(const char* str) {
    return static_cast<size_t>(std::stoul(str));
}

std::string join_path(const std::string& parent, const std::string& filename) {
    if (parent.empty()) return filename;
    if (parent.back() == '/' || parent.back() == '\\') return parent + filename;
    return parent + "/" + filename;
}

std::vector<Object> load_objects(const std::vector<std::string>& fpaths, std::string parent_path) {
    // Loads objects from a list of obj file paths.

    std::vector<Object> objects;

    for (const auto& filename : fpaths) {
        std::string file_path = join_path(parent_path, filename);
        std::ifstream file(file_path);
        if (!file) {
            std::cerr << "Error: Could not open file " << file_path << std::endl;
            continue;
        }

        std::vector<Vertex> vertices{{0.0, 0.0, 0.0}};
        std::vector<Normal> normals{{0.0, 0.0, 0.0}};
        std::vector<Face> faces;
        std::string line;

        // Parse all lines in an obj file
        while (std::getline(file, line)) {
            std::string s = line;
            // Skip whitespace, blank lines, and comments
            auto p = s.find_first_not_of(" \t\r\n");
            if (p == std::string::npos) continue;
            if (s[p] == '#') continue;

            std::istringstream line_stream(s);
            std::string type;
            line_stream >> type;

            if (type != "v" && type != "vn" && type != "f") {
                throw std::runtime_error("Invalid format: must start with 'v', 'vn', or 'f'");
            }

            if (type == "v") {
                double x, y, z;
                if (!(line_stream >> x >> y >> z)) throw std::runtime_error("Invalid Vertex format");
                std::string leftover;
                if (line_stream >> leftover) throw std::runtime_error("Extra data in Vertex");
                vertices.push_back({x, y, z});
            } 
            else if (type == "vn") {
                double x, y, z;
                if (!(line_stream >> x >> y >> z)) throw std::runtime_error("Invalid normal format");
                std::string leftover;
                if (line_stream >> leftover) throw std::runtime_error("Extra data in normal");
                normals.push_back({x, y, z});
            } 
            else if (type == "f") {
                unsigned int v_idx[3], n_idx[3];
                for (int i = 0; i < 3; ++i) {
                    std::string token;
                    if (!(line_stream >> token)) throw std::runtime_error("Invalid face format");

                    auto pos = token.find("//");
                    if (pos == std::string::npos) throw std::runtime_error("Expected 'v//vn' format");

                    v_idx[i] = std::stoi(token.substr(0, pos));
                    n_idx[i] = std::stoi(token.substr(pos + 2));
                }

                // check bounds
                for (int i = 0; i < 3; ++i) {
                    if (v_idx[i] >= vertices.size() || n_idx[i] >= normals.size()) {
                        throw std::runtime_error("Face index out of range");
                    }
                }

                faces.push_back({v_idx[0], v_idx[1], v_idx[2], n_idx[0], n_idx[1], n_idx[2]});
            }
        }

        objects.push_back({file_path, vertices, normals, faces});
    }

    return objects;
}

Matrix4d make_transform_from_lines(const std::vector<std::string>& lines) {
    // In scene file, makes a transform matrix from a series of transformations applied to an Object

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
            T = make_translation(tx, ty, tz);
        } else if (kind == 's') {
            double sx, sy, sz;
            if (!(iss >> sx >> sy >> sz)) {
                throw std::runtime_error("Invalid scale at line " + std::to_string(lineno));
            }
            T = make_scaling(sx, sy, sz);
        } else if (kind == 'r') {
            double rx, ry, rz, angle;
            if (!(iss >> rx >> ry >> rz >> angle)) {
                throw std::runtime_error("Invalid rotation at line " + std::to_string(lineno));
            }
            T = make_rotation(rx, ry, rz, angle);
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
    const std::string& name, const std::unordered_map<std::string,size_t>& name_to_idx
) {
    // Gets the index of an Object name in the objects list
    auto it = name_to_idx.find(name);
    if (it == name_to_idx.end()) {
        throw std::out_of_range("Name not found: " + name);
    }
    return it->second;
}

std::size_t parse_Object_mappings(
    const std::vector<std::string>& lines,
    std::vector<std::string>& Object_names,
    std::vector<std::string>& Object_paths
) {
    // Gets pairs of Object names and their respective obj files, returns the line number where this section ends, e.g.:
        // objects:
        // cube cube.obj
        // bunny bunny.obj

    bool started_mapping = false;
    std::size_t i = 0;

    for (; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        auto first_non_ws = line.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) { // First blank after mapping starts is end of mapping section
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
        Object_names.push_back(name);
        Object_paths.push_back(path);
    }
    return i; // index of first line after the mapping section (roughly where transforms begin)
}

void process_transform_blocks(
    const std::vector<std::string>& lines,
    std::size_t start_idx,
    std::vector<Object>& objects,
    const std::vector<std::string>& Object_names,
    const std::unordered_map<std::string, std::size_t>& name_to_idx,
    std::vector<ObjectInstance>& out_transformed
) {
    // Each block defines an Object and a series of transformations to apply to it, e.g.:
        // bunny
        // ambient 0.2 0.1 0.2
        // diffuse 0.6969 0.0327293 0.626496
        // specular 0.1 0.1 0.1
        // shininess 0.2
        // s 1 1 1
        // t 0.4 -0.9 0
    // This function processes all of those blocks and writes the Object instances into "out_transformed"
    // with lighting saved and after transforming them from Object space to world coordinates.

    if (objects.size() != Object_names.size()) {
        throw std::runtime_error("Loaded different number of objects and names.");
    }

    std::unordered_map<std::string, std::size_t> copy_count;

    std::string current_name;
    std::vector<std::string> current_transform_lines;

    // material for the current block (defaults)
    Eigen::Vector3d current_ambient = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_diffuse = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_specular = Eigen::Vector3d::Zero();
    double current_shininess = 0.0;

    // Call this when we reach the end of one Object block, so process the accumulated lines and save the Object, clear, and continue.
    auto flush_instance = [&]() {
        if (current_name.empty() || current_transform_lines.empty()) {
            current_transform_lines.clear(); // Nothing to process
            return;
        }
        try {
            std::size_t base_idx = find_string_idx(current_name, name_to_idx);
            Eigen::Matrix4d M = make_transform_from_lines(current_transform_lines);
            auto base = objects.at(base_idx); // copy
            apply_transform_to_object(base, M); // transform the copy
            std::size_t n = ++copy_count[current_name];
            std::string out_name = current_name + "_copy" + std::to_string(n);
            out_transformed.emplace_back(ObjectInstance{
                std::move(base), out_name,
                current_ambient, current_diffuse,
                current_specular, current_shininess
            });
        } catch (const std::exception& e) {
            std::cerr << "Error processing block for '" << current_name << "': " << e.what() << std::endl;
        }
        
        // Reset the block
        current_name.clear();
        current_transform_lines.clear();
        current_ambient.setZero();
        current_diffuse.setZero();
        current_specular.setZero();
        current_shininess = 0.0;
    };

    for (std::size_t i = start_idx; i < lines.size(); ++i) {
        const std::string& tline = lines[i];
        auto first_non_ws = tline.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) { 
            // A blank line means we are done with this Object instance
            flush_instance();
            continue;
        }
        if (tline[first_non_ws] == '#') continue; // allow comments

        std::istringstream iss(tline.substr(first_non_ws));
        std::string tok;
        iss >> tok;
        if (!iss && tok.empty()) continue;

        // lighting
        if (tok == "ambient") {
            if (current_name.empty()) {
                std::cerr << "Warning: 'ambient' before Object name, skipping line " << tline << std::endl;
                continue;
            }
            double x=0, y=0, z=0; iss >> x >> y >> z;
            current_ambient = Eigen::Vector3d(x, y, z);
            continue;
        } else if (tok == "diffuse") {
            if (current_name.empty()) {
                std::cerr << "Warning: 'diffuse' before Object name, skipping line " << tline << std::endl;
                continue;
            }
            double x=0, y=0, z=0; iss >> x >> y >> z;
            current_diffuse = Eigen::Vector3d(x, y, z);
            continue;
        } else if (tok == "specular") {
            if (current_name.empty()) {
                std::cerr << "Warning: 'specular' before Object name, skipping line " << tline << std::endl;
                continue;
            }
            double x=0, y=0, z=0; iss >> x >> y >> z;
            current_specular = Eigen::Vector3d(x, y, z);
            continue;
        } else if (tok == "shininess") {
            if (current_name.empty()) {
                std::cerr << "Warning: 'shininess' before Object name, skipping line " << tline << std::endl;
                continue;
            }
            double s=0; iss >> s;
            current_shininess = s;
            continue;
        }

        // transforms
        if (tok == "t" || tok == "r" || tok == "s") {
            if (current_name.empty()) {
                std::cerr << "Warning: Transform before Object name, skipping line " << tline << std::endl;
                continue;
            }
            current_transform_lines.push_back(tline.substr(first_non_ws));
            continue;
        } else {
            flush_instance();
            current_name = tok;
        }
    }

    flush_instance(); // Process what remains after we run out of lines
}

std::vector<ObjectInstance> make_transformed_objects_from_lines(const std::vector<std::string>& lines, std::string parent_path) {
    // Parse mapping
    std::vector<std::string> Object_names;
    std::vector<std::string> Object_paths;
    std::size_t next_idx = parse_Object_mappings(lines, Object_names, Object_paths);

    // Load base objects in the same order
    std::vector<Object> objects = load_objects(Object_paths, parent_path);

    // Build name to index mapping
    std::unordered_map<std::string, std::size_t> name_to_idx;
    name_to_idx.reserve(Object_names.size());
    for (std::size_t i = 0; i < Object_names.size(); ++i) {
        name_to_idx.emplace(Object_names[i], i);
    }

    // Process transform blocks
    std::vector<ObjectInstance> transformed_objects;
    process_transform_blocks(
        lines,
        next_idx,
        objects,
        Object_names,
        name_to_idx,
        transformed_objects
    );

    return transformed_objects;
}

void read_cam_params_and_lights(CameraParams& cam, std::vector<Light>& lights, std::ifstream& fin){
    // Parse the opening lines of a scene file which defines the camera setup and lights, e.g.:
        // camera:
        // position 0 0 5
        // orientation 0 1 0 0
        // near 1
        // far 10
        // left -0.5
        // right 0.5
        // top 0.5
        // bottom -0.5

        // light -2 2 2 , 1 1 1 , 0.2
        // light 2 0 2 , 0 0 1 , 0.8

    std::string raw;
    bool in_objects = false;
    bool in_lights = false;

    while (std::getline(fin, raw)) {
        // Skip empty lines and comments
        auto first_non_ws = raw.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (raw[first_non_ws] == '#') continue;
        std::string line = raw.substr(first_non_ws);

        // Check if we hit the next section, return if so
        if (line == "objects:") { in_objects = true; break; }

        // parse "key ..." lines
        std::istringstream iss(line);
        std::string key; iss >> key;

        if (key == "light") {
            in_lights = true;
            double x, y, z, r, g, b, atten;
            char comma;
            if (!(iss >> x >> y >> z >> comma >> r >> g >> b >> comma >> atten)) {
                throw std::runtime_error("Invalid light format");
            }
            lights.push_back({x, y, z, r, g, b, atten});
        } else if (in_lights) {
            std::cerr << "Note: invalid key in lights section '" << key << "'.\n";
        }

        if (!in_lights) {
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
    }

    if (cam.znear==0 || cam.zfar==cam.znear ||
        cam.right==cam.left || cam.top==cam.bottom) {
        throw std::runtime_error("Invalid frustum parameters");
    }

    if (!in_objects) {
        std::cerr << "Missing 'objects:' after camera section.\n";
    }
}

Scene parse_scene_file(std::ifstream& fin, std::string parent_path){
    CameraParams cam;
    std::vector<std::string> Object_section_lines;

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

    // 2) Read camera and light fields until we see "objects:"
    std::vector<Light> lights;
    read_cam_params_and_lights(cam, lights, fin);

    // 3) Collect everything under "objects:" as lines (left-trimmed)
    while (std::getline(fin, raw)) {
        Object_section_lines.push_back(raw);
    }

    // 4) Get transformed objects from "objects:" section
    std::vector<ObjectInstance> scene_objects = make_transformed_objects_from_lines(Object_section_lines, parent_path);

    return Scene({make_cam_matrices(cam), scene_objects, lights});
}

void write_ppm(const Image& img){
    std::cout << "P3\n" << img.xres << " " << img.yres << "\n255\n";
    for (size_t i = 0; i < img.xres * img.yres * 3; i += 3) {
        std::cout << static_cast<int>(img.img[i]) << " "
                << static_cast<int>(img.img[i+1]) << " "
                << static_cast<int>(img.img[i+2]) << "\n";
    }
}