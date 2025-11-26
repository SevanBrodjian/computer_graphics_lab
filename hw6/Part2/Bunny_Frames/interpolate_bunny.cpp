/* CS/CNS 171 -- HW6 Part 2
 * Interpolates missing bunny frames using Catmull-Rom splines.
 */
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

template <typename T>
T catmullRom(const T &p0, const T &p1, const T &p2, const T &p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

struct ObjData {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::string> otherLines;
};

ObjData readObj(const std::string &filename) {
    ObjData data;
    std::ifstream in(filename.c_str());
    std::string line;
    while (std::getline(in, line)) {
        if (line.size() > 1 && line[0] == 'v' && line[1] == ' ') {
            std::stringstream ss(line);
            char v;
            Eigen::Vector3d vert;
            ss >> v >> vert[0] >> vert[1] >> vert[2];
            data.vertices.push_back(vert);
        } else {
            data.otherLines.push_back(line);
        }
    }
    return data;
}

void writeObj(const std::string &filename, const std::vector<Eigen::Vector3d> &vertices,
              const std::vector<std::string> &otherLines) {
    std::ofstream out(filename.c_str());
    out << std::fixed << std::setprecision(6);
    for (const auto &v : vertices) {
        out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    }
    for (const auto &line : otherLines) {
        out << line << "\n";
    }
}

Eigen::Vector3d interpolateVertex(int frame, const std::map<int, std::vector<Eigen::Vector3d>> &keyframes,
                                  int index) {
    int f0, f1, f2, f3;
    if (frame < 5) {
        f0 = 0; f1 = 0; f2 = 5; f3 = 10;
    } else if (frame < 10) {
        f0 = 0; f1 = 5; f2 = 10; f3 = 15;
    } else if (frame < 15) {
        f0 = 5; f1 = 10; f2 = 15; f3 = 20;
    } else {
        f0 = 10; f1 = 15; f2 = 20; f3 = 20;
    }

    double t = static_cast<double>(frame - f1) / static_cast<double>(f2 - f1);
    const Eigen::Vector3d &p0 = keyframes.at(f0)[index];
    const Eigen::Vector3d &p1 = keyframes.at(f1)[index];
    const Eigen::Vector3d &p2 = keyframes.at(f2)[index];
    const Eigen::Vector3d &p3 = keyframes.at(f3)[index];

    return catmullRom(p0, p1, p2, p3, t);
}

int main() {
    std::vector<int> frameIds = {0, 5, 10, 15, 20};
    std::map<int, std::vector<Eigen::Vector3d>> keyframes;
    std::vector<std::string> otherLines;

    for (int id : frameIds) {
        std::stringstream ss;
        ss << "keyframes/bunny" << std::setfill('0') << std::setw(2) << id << ".obj";
        ObjData data = readObj(ss.str());
        if (keyframes.empty()) {
            otherLines = data.otherLines;
        }
        keyframes[id] = data.vertices;
    }

    if (keyframes.empty()) {
        std::cerr << "Failed to load keyframes." << std::endl;
        return 1;
    }

    mkdir("generated_frames", 0777);

    std::vector<int> targets = {1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19};
    size_t vertexCount = keyframes.begin()->second.size();

    for (int frame : targets) {
        std::vector<Eigen::Vector3d> verts(vertexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            verts[i] = interpolateVertex(frame, keyframes, static_cast<int>(i));
        }

        std::stringstream ss;
        ss << "generated_frames/bunny" << std::setfill('0') << std::setw(2) << frame << ".obj";
        writeObj(ss.str(), verts, otherLines);
        std::cout << "Wrote frame " << frame << std::endl;
    }

    return 0;
}
