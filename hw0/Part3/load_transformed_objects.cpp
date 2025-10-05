// main.cpp
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


// #############################################
// # Part 1
// #############################################

// Change to doubles for Eigen compatibility
struct vertex {
    double x;
    double y;
    double z;
};

struct face {
    unsigned int v1;
    unsigned int v2;
    unsigned int v3;
};

struct object {
    std::string filename;
    std::vector<vertex> vertices;
    std::vector<face> faces;

    void print(){
        std::cout << filename << ":" << std::endl << std::endl;

        for (std::size_t i = 1; i < vertices.size(); ++i) std::cout << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << std::endl;
        for (const auto &f : faces) std::cout << "f " << f.v1 << " " << f.v2 << " " << f.v3 << std::endl;
    }
};

std::vector<object> load_objects(const std::vector<std::string>& fpaths) {
    std::vector<object> objects;

    for (const auto& filename : fpaths) {
        std::ifstream file(filename);
        if (!file) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            continue;
        }

        std::vector<vertex> vertices{{0.0, 0.0, 0.0}};
        std::vector<face> faces;
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#' || std::isspace(line[0])) continue;

            char type = line[0];
            std::istringstream iss(line.substr(1));

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
                faces.push_back({a, b, c});
            }
        }

        objects.push_back({filename, vertices, faces});
    }

    return objects;
}

// #############################################
// # Part 2
// #############################################

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

// From Part 2, modified to take a vector of transform lines instead of files
// Assumption: We want to left-multiply by M, not M inverse
// This makes sense if we are interpreting the file as applying the transformations in order, directly to the vertices
// However, the assignment isn't toally clear on this, why did we compute M inverse in part 2?
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

// #############################################
// # Part 3
// #############################################

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

object applyTransformToObject(const object& src, const Matrix4d& M) {
    object out = src;
    for (size_t vi = 1; vi < out.vertices.size(); ++vi) {
        auto& v = out.vertices[vi];
        Eigen::Vector4d p(v.x, v.y, v.z, 1.0);
        Eigen::Vector4d q = M * p;
        v.x = q[0];
        v.y = q[1];
        v.z = q[2];
    }
    return out;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " object_transforms.txt\n";
        return 1;
    }

    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cerr << "Error: could not open file: " << argv[1] << std::endl;
        return 1;
    }

    // STEP 1: Load in object files and store
    std::vector<std::string> object_names;
    std::vector<std::string> object_paths;

    std::string line;
    bool started_mapping = false;
    while (std::getline(fin, line)) {
        // Trim leading whitespace
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            if (started_mapping) break;
            else continue;
        }
        if (line[0] == '#') continue;

        started_mapping = true;
        std::istringstream iss(line);
        std::string name, path;
        if (!(iss >> name >> path)) {
            std::cerr << "Warning: Cannot read line: " << line << std::endl;
            continue;
        }
        object_names.push_back(name);
        object_paths.push_back(path);
    }

    // Keep objects in the same order as object_names
    std::vector<object> objects = load_objects(object_paths);

    // Build name to index map
    std::unordered_map<std::string, size_t> name_to_idx;
    name_to_idx.reserve(object_names.size());
    for (size_t i = 0; i < object_names.size(); ++i) {
        name_to_idx.emplace(object_names[i], i);
    }


    // STEP 2: Compute transformation matrices for each object listed (possible repeats)

    std::vector<object> transformed_objects;
    std::vector<std::string> transformed_object_names;
    std::unordered_map<std::string, size_t> copy_count;

    if (objects.size() != object_names.size()) {
        std::cerr << "Error: Loaded different number of objects and names.\n";
        return 1;
    }

    std::string tline;
    std::string current_name;
    std::vector<std::string> current_transform_lines;

    // Lambda to process blocks
    auto flush_block = [&](){
        if (current_name.empty() || current_transform_lines.empty()) { 
            current_transform_lines.clear(); 
            return; 
        }
        try {
            size_t base_idx = find_string_idx(current_name, name_to_idx);
            Matrix4d M = makeTransformFromLines(current_transform_lines);
            object transformed = applyTransformToObject(objects.at(base_idx), M);
            size_t n = ++copy_count[current_name];
            transformed_object_names.push_back(current_name + "_copy" + std::to_string(n));
            transformed_objects.push_back(std::move(transformed));
        } catch (const std::exception& e) {
            std::cerr << "Error processing block for '" << current_name << "': " << e.what() << std::endl;
        }
        current_transform_lines.clear();
        current_name.clear();
    };

    // Continue from the current position in fin (after object declarations)
    while (std::getline(fin, tline)) {
        // Again, skip whitespace and comments to find the next block corresponding to an object and its transformations
        auto first_non_ws = tline.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) { flush_block(); continue; }
        if (tline[first_non_ws] == '#') continue;

        std::istringstream iss(tline.substr(first_non_ws));
        std::string tok;
        iss >> tok; // Gets string up to next whitespace
        if (!iss) continue;

        if (tok == "t" || tok == "r" || tok == "s") {
            if (current_name.empty()) {
                std::cerr << "Warning: Transform before object name, skipping line " << tline << std::endl;
                continue;
            }
            current_transform_lines.push_back(tline.substr(first_non_ws));
        } else {
            flush_block();              // finish previous block
            current_name = tok;         // start a new one
        }
    }
    flush_block();
    fin.close();

    // Print results
    for (size_t i = 0; i < transformed_objects.size(); ++i) {
        std::cout << transformed_object_names[i] << std::endl;
        const std::vector<vertex>& verts = transformed_objects[i].vertices;
        for (std::size_t i = 1; i < verts.size(); ++i) {
            std::cout << "v " << verts[i].x << " " << verts[i].y << " " << verts[i].z << std::endl;
        }
        if (i != transformed_objects.size() - 1) std::cout << std::endl;
    }
}