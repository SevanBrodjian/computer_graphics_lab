#include "scene_loader.h"

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

// Reused from hw2

namespace {
std::string join_path(const std::string& parent, const std::string& filename) {
    if (parent.empty()) return filename;
    if (parent.back() == '/' || parent.back() == '\\') return parent + filename;
    return parent + "/" + filename;
}

Matrix4d make_translation(double tx, double ty, double tz) {
    Matrix4d T = Matrix4d::Identity();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    return T;
}

Matrix4d make_scaling(double sx, double sy, double sz) {
    Matrix4d S = Matrix4d::Identity();
    S(0, 0) = sx;
    S(1, 1) = sy;
    S(2, 2) = sz;
    return S;
}

Matrix4d make_rotation(double rx, double ry, double rz, double angle) {
    Vector3d axis(rx, ry, rz);
    if (axis.norm() == 0.0) {
        return Matrix4d::Identity();
    }
    axis.normalize();
    Matrix3d R3 = AngleAxisd(angle, axis).toRotationMatrix();
    Matrix4d R = Matrix4d::Identity();
    R.block<3,3>(0,0) = R3;
    return R;
}

void apply_transform_to_object(Object& src, const Matrix4d& M, bool transform_normals = true) {
    for (std::size_t vi = 1; vi < src.vertices.size(); ++vi) {
        auto& v = src.vertices[vi];
        Eigen::Vector4d p(v.x, v.y, v.z, 1.0);
        Eigen::Vector3d q = (M * p).hnormalized();
        v.x = q[0];
        v.y = q[1];
        v.z = q[2];
    }

    if (transform_normals) {
        const Matrix3d A = M.block<3,3>(0,0);
        Matrix3d N;
        double det = A.determinant();
        if (std::abs(det) < 1e-15) {
            N = Matrix3d::Identity();
        } else {
            N = A.inverse().transpose();
        }

        for (std::size_t ni = 1; ni < src.normals.size(); ++ni) {
            auto& vn = src.normals[ni];
            Eigen::Vector3d n(vn.x, vn.y, vn.z);
            n = N * n;
            n.normalize();
            vn.x = n.x();
            vn.y = n.y();
            vn.z = n.z();
        }
    }
}

Matrix4d make_transform_from_lines(const std::vector<std::string>& lines) {
    Matrix4d M = Matrix4d::Identity();
    std::size_t lineno = 0;

    for (const auto& raw_line : lines) {
        ++lineno;
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
        }

        M = T * M;
    }

    return M;
}

std::size_t find_string_idx(
    const std::string& name,
    const std::unordered_map<std::string, std::size_t>& name_to_idx) {
    auto it = name_to_idx.find(name);
    if (it == name_to_idx.end()) {
        throw std::out_of_range("Name not found: " + name);
    }
    return it->second;
}

std::vector<Object> load_objects(const std::vector<std::string>& fpaths,
                                 const std::string& parent_path) {
    std::vector<Object> objects;
    for (const auto& filename : fpaths) {
        std::string file_path = join_path(parent_path, filename);
        std::ifstream file(file_path);
        if (!file) {
            throw std::runtime_error("Error: Could not open file " + file_path);
        }

        std::vector<Vertex> vertices{{0.0, 0.0, 0.0}};
        std::vector<Normal> normals{{0.0, 0.0, 0.0}};
        std::vector<Face> faces;
        std::string line;

        while (std::getline(file, line)) {
            auto first_non_ws = line.find_first_not_of(" \t\r\n");
            if (first_non_ws == std::string::npos) continue;
            if (line[first_non_ws] == '#') continue;

            std::istringstream line_stream(line.substr(first_non_ws));
            std::string type;
            line_stream >> type;

            if (type == "v") {
                double x, y, z;
                if (!(line_stream >> x >> y >> z)) {
                    throw std::runtime_error("Invalid vertex format");
                }
                vertices.push_back({x, y, z});
            } else if (type == "vn") {
                double x, y, z;
                if (!(line_stream >> x >> y >> z)) {
                    throw std::runtime_error("Invalid normal format");
                }
                normals.push_back({x, y, z});
            } else if (type == "f") {
                unsigned int v_idx[3], n_idx[3];
                for (int i = 0; i < 3; ++i) {
                    std::string token;
                    if (!(line_stream >> token)) {
                        throw std::runtime_error("Invalid face format");
                    }
                    auto pos = token.find("//");
                    if (pos == std::string::npos) {
                        throw std::runtime_error("Expected 'v//vn' format");
                    }
                    v_idx[i] = static_cast<unsigned int>(std::stoul(token.substr(0, pos)));
                    n_idx[i] = static_cast<unsigned int>(std::stoul(token.substr(pos + 2)));
                }
                faces.push_back({v_idx[0], v_idx[1], v_idx[2], n_idx[0], n_idx[1], n_idx[2]});
            }
        }

        objects.push_back({file_path, std::move(vertices), std::move(normals), std::move(faces)});
    }

    return objects;
}

std::size_t parse_object_mappings(const std::vector<std::string>& lines,
                                  std::vector<std::string>& object_names,
                                  std::vector<std::string>& object_paths) {
    bool started_mapping = false;
    std::size_t i = 0;

    for (; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        auto first_non_ws = line.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) {
            if (started_mapping) { ++i; break; }
            continue;
        }
        if (line[first_non_ws] == '#') continue;

        started_mapping = true;
        std::istringstream iss(line.substr(first_non_ws));
        std::string name, path;
        if (!(iss >> name >> path)) {
            throw std::runtime_error("Invalid object mapping: " + line);
        }
        object_names.push_back(name);
        object_paths.push_back(path);
    }

    return i;
}

void process_transform_blocks(const std::vector<std::string>& lines,
                              std::size_t start_idx,
                              std::vector<Object>& objects,
                              const std::vector<std::string>& object_names,
                              const std::unordered_map<std::string, std::size_t>& name_to_idx,
                              std::vector<ObjectInstance>& out_transformed) {
    if (objects.size() != object_names.size()) {
        throw std::runtime_error("Mismatched object counts");
    }

    std::unordered_map<std::string, std::size_t> copy_count;
    std::string current_name;
    std::vector<std::string> current_transform_lines;
    Eigen::Vector3d current_ambient = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_diffuse = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_specular = Eigen::Vector3d::Zero();
    double current_shininess = 0.0;

    auto flush_instance = [&]() {
        if (current_name.empty() || current_transform_lines.empty()) {
            current_transform_lines.clear();
            return;
        }
        std::size_t base_idx = find_string_idx(current_name, name_to_idx);
        Matrix4d M = make_transform_from_lines(current_transform_lines);
        auto base = objects.at(base_idx);
        apply_transform_to_object(base, M);
        std::size_t n = ++copy_count[current_name];
        std::string out_name = current_name + "_copy" + std::to_string(n);
        out_transformed.emplace_back(ObjectInstance{std::move(base), out_name,
            current_ambient, current_diffuse, current_specular, current_shininess});
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
            flush_instance();
            continue;
        }
        if (tline[first_non_ws] == '#') continue;

        std::istringstream iss(tline.substr(first_non_ws));
        std::string tok;
        iss >> tok;
        if (!iss && tok.empty()) continue;

        if (tok == "ambient") {
            if (current_name.empty()) continue;
            double x=0, y=0, z=0; iss >> x >> y >> z;
            current_ambient = Eigen::Vector3d(x, y, z);
            continue;
        } else if (tok == "diffuse") {
            if (current_name.empty()) continue;
            double x=0, y=0, z=0; iss >> x >> y >> z;
            current_diffuse = Eigen::Vector3d(x, y, z);
            continue;
        } else if (tok == "specular") {
            if (current_name.empty()) continue;
            double x=0, y=0, z=0; iss >> x >> y >> z;
            current_specular = Eigen::Vector3d(x, y, z);
            continue;
        } else if (tok == "shininess") {
            if (current_name.empty()) continue;
            double s = 0; iss >> s;
            current_shininess = s;
            continue;
        }

        if (tok == "t" || tok == "r" || tok == "s") {
            if (current_name.empty()) continue;
            current_transform_lines.push_back(tline.substr(first_non_ws));
            continue;
        }

        flush_instance();
        current_name = tok;
    }

    flush_instance();
}

void read_cam_params_and_lights(CameraParams& cam, std::vector<Light>& lights, std::ifstream& fin) {
    std::string raw;
    while (std::getline(fin, raw)) {
        auto first_non_ws = raw.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (raw[first_non_ws] == '#') continue;
        std::string line = raw.substr(first_non_ws);

        if (line == "objects:") {
            break;
        }

        std::istringstream iss(line);
        std::string key; iss >> key;

        if (key == "light") {
            double x, y, z, r, g, b, atten;
            char comma;
            if (!(iss >> x >> y >> z >> comma >> r >> g >> b >> comma >> atten)) {
                throw std::runtime_error("Invalid light format");
            }
            lights.push_back({x, y, z, r, g, b, atten});
        } else if (key == "position") {
            iss >> cam.px >> cam.py >> cam.pz;
        } else if (key == "orientation") {
            iss >> cam.ox >> cam.oy >> cam.oz >> cam.oang;
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
        }
    }

    if (cam.znear == 0 || cam.zfar == cam.znear ||
        cam.right == cam.left || cam.top == cam.bottom) {
        throw std::runtime_error("Invalid frustum parameters");
    }
}

Camera make_cam_matrices(const CameraParams& cam) {
    Matrix4d T_C = make_translation(cam.px, cam.py, cam.pz);
    Matrix4d R_C = make_rotation(cam.ox, cam.oy, cam.oz, cam.oang);
    Matrix4d C_inv = (T_C * R_C).inverse();

    double n = cam.znear;
    double f = cam.zfar;
    double l = cam.left;
    double r = cam.right;
    double b = cam.bottom;
    double t = cam.top;

    Matrix4d P;
    P <<
        (2 * n) / (r - l),  0,                  (r + l) / (r - l),   0,
        0,                  (2 * n) / (t - b),  (t + b) / (t - b),   0,
        0,                  0,                  -(f + n) / (f - n),  -(2 * f * n) / (f - n),
        0,                  0,                  -1,                  0;

    return Camera{C_inv, P};
}

} // namespace

std::string parse_parent_path(const std::string& path) {
    std::size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return "";
    return path.substr(0, pos);
}

std::size_t parse_size_t(const char* str) {
    return static_cast<std::size_t>(std::stoul(str));
}

Scene parse_scene_file(std::ifstream& fin, const std::string& parent_path) {
    CameraParams cam;
    std::vector<std::string> object_section_lines;

    std::string raw;
    bool in_camera = false;

    while (std::getline(fin, raw)) {
        auto first_non_ws = raw.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (raw[first_non_ws] == '#') continue;
        std::string line = raw.substr(first_non_ws);
        if (line == "camera:") { in_camera = true; break; }
    }
    if (!in_camera) {
        throw std::runtime_error("Missing 'camera:' section");
    }

    std::vector<Light> lights;
    read_cam_params_and_lights(cam, lights, fin);

    while (std::getline(fin, raw)) {
        object_section_lines.push_back(raw);
    }

    std::vector<std::string> object_names;
    std::vector<std::string> object_paths;
    std::size_t next_idx = parse_object_mappings(object_section_lines, object_names, object_paths);
    std::vector<Object> objects = load_objects(object_paths, parent_path);

    std::unordered_map<std::string, std::size_t> name_to_idx;
    name_to_idx.reserve(object_names.size());
    for (std::size_t i = 0; i < object_names.size(); ++i) {
        name_to_idx.emplace(object_names[i], i);
    }

    std::vector<ObjectInstance> transformed_objects;
    process_transform_blocks(object_section_lines, next_idx, objects,
        object_names, name_to_idx, transformed_objects);

    return Scene{make_cam_matrices(cam), std::move(transformed_objects), std::move(lights)};
}

