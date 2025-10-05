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

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;


// #############################################
// # Part 1
// #############################################

struct vertex {
    float x;
    float y;
    float z;
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

        std::vector<vertex> vertices{{0.0f, 0.0f, 0.0f}};
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
                float x, y, z;
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

// From Part 2
Matrix4d makeInverseTransformFromLines(const std::vector<std::string>& lines) {
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

    return M.inverse();
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

object applyTransformToObject(const object& src, const Matrix4d& Minv) {
    object out = src; // copy
    for (size_t vi = 1; vi < out.vertices.size(); ++vi) {
        Eigen::Vector4d p(out.vertices[vi].x, out.vertices[vi].y, out.vertices[vi].z, 1.0);
        Eigen::Vector4d q = Minv * p;
        out.vertices[vi].x = static_cast<float>(q[0]);
        out.vertices[vi].y = static_cast<float>(q[1]);
        out.vertices[vi].z = static_cast<float>(q[2]);
    }
    return out;
};

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
    fin.close();

    // Keep objects in the same order as object_names
    std::vector<object> objects = load_objects(object_paths);

    // Build name to index map
    std::unordered_map<std::string, size_t> name_to_idx;
    name_to_idx.reserve(object_names.size());
    for (size_t i = 0; i < object_names.size(); ++i) {
        name_to_idx.emplace(object_names[i], i);
    }

    // Quick check
    // std::cout << "Parsed objects:\n";
    // for (size_t i = 0; i < object_names.size(); ++i) {
    //     std::cout << "  " << object_names[i] << " -> " << object_paths[i] << "\n";
    //     objects[i].print();
    // }

    // // Example usage:
    // try {
    //     std::string obj_query = "tet2";
    //     size_t idx = find_string_idx(obj_query, name_to_idx);
    //     const object& obj = objects.at(idx);
    //     std::cout << "Object " << obj_query << " at " << idx << std::endl;
    // } catch (const std::out_of_range& e) {
    //     std::cerr << e.what() << "\n";
    // }

    
    // STEP 2: Compute transformation matrices for each object listed (possible repeats)

    std::vector<object> transformed_objects;
    std::vector<std::string> transformed_object_names;
    std::unordered_map<std::string, size_t> copy_count;

    std::ifstream fin2(argv[1]);

    if (!fin2) {
        std::cerr << "Error: could not re-open file for transforms: " << argv[1] << std::endl;
        return 1;
    }
    {
        std::string l;
        bool started_mapping2 = false;
        while (std::getline(fin2, l)) {
            auto first_non_ws = l.find_first_not_of(" \t\r\n");
            if (first_non_ws == std::string::npos) {
                if (started_mapping2) break;   // blank line ends mapping section
                else continue;                 // skip leading blanks
            }
            if (l[first_non_ws] == '#') continue; // skip comments
            started_mapping2 = true;              // reading mapping lines
            // do nothing else; just advance until the first blank line after mapping
        }
    }

    // Helpers local to STEP 2
    std::string current_name;
    std::vector<std::string> current_transform_lines;

    auto flush_block = [&]() {
        if (current_name.empty() || current_transform_lines.empty()) {
            current_transform_lines.clear();
            return;
        }

        try {
            // find base object index
            size_t base_idx = find_string_idx(current_name, name_to_idx);

            // build inverse transform from the collected lines
            Matrix4d Minv = makeInverseTransformFromLines(current_transform_lines);

            // transform and store
            object transformed = applyTransformToObject(objects.at(base_idx), Minv);

            // name as _copyN (1-indexed per base name)
            size_t n = ++copy_count[current_name];
            transformed_object_names.push_back(current_name + "_copy" + std::to_string(n));
            transformed_objects.push_back(std::move(transformed));
        } catch (const std::exception& e) {
            std::cerr << "Error processing block for '" << current_name << "': " << e.what() << "\n";
        }

        current_transform_lines.clear();
        current_name.clear();
    };

    // Read the transform section
    std::string tline;
    while (std::getline(fin2, tline)) {
        auto first_non_ws = tline.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) {
            // blank line ends current block
            flush_block();
            continue;
        }
        if (tline[first_non_ws] == '#') continue; // skip comments

        char c = tline[first_non_ws];
        if (c == 't' || c == 'r' || c == 's') {
            if (current_name.empty()) {
                std::cerr << "Warning: transform before object name; skipping: " << tline << "\n";
                continue;
            }
            current_transform_lines.push_back(tline.substr(first_non_ws));
        } else {
            // new object name line (first token is the name)
            flush_block(); // finish previous block if any
            std::istringstream iss(tline.substr(first_non_ws));
            std::string name_token;
            if (!(iss >> name_token)) {
                std::cerr << "Warning: malformed object name line; skipping: " << tline << "\n";
                continue;
            }
            current_name = name_token;
        }
    }
    // Flush last block at EOF
    flush_block();

    fin2.close();

    // (Optional) Print results
    for (size_t i = 0; i < transformed_objects.size(); ++i) {
        std::cout << "Transformed: " << transformed_object_names[i] << "\n";
        transformed_objects[i].print();
    }
}