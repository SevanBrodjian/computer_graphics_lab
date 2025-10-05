#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

static Matrix4d makeTranslation(double tx, double ty, double tz) {
    Matrix4d T = Matrix4d::Identity();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    return T;
}

static Matrix4d makeScaling(double sx, double sy, double sz) {
    Matrix4d S = Matrix4d::Identity();
    S(0, 0) = sx;
    S(1, 1) = sy;
    S(2, 2) = sz;
    return S;
}

static Matrix4d makeRotation(double rx, double ry, double rz, double angle) {
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

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " transforms.txt\n";
        return 1;
    }
    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cerr << "Error: could not open file: " << argv[1] << "\n";
        return 1;
    }

    Matrix4d M = Matrix4d::Identity(); // will hold the product

    std::string line;
    size_t lineno = 0;
    while (std::getline(fin, line)) {
        ++lineno;
        // Skip empty lines and comments
        auto first_non_ws = line.find_first_not_of(" \t\r\n");
        if (first_non_ws == std::string::npos) continue;
        if (line[first_non_ws] == '#') continue;

        std::istringstream iss(line);
        char kind;
        iss >> kind;

        if (!iss) {
            std::cerr << "Error: Couldn't read line " << lineno << std::endl;
            continue;
        }

        Matrix4d T = Matrix4d::Identity();

        if (kind == 't') {
            double tx, ty, tz;
            if (!(iss >> tx >> ty >> tz)) {
                std::cerr << "Error: Problem in line " << lineno << std::endl;
                return 1;
            }
            T = makeTranslation(tx, ty, tz);
        } else if (kind == 's') {
            double sx, sy, sz;
            if (!(iss >> sx >> sy >> sz)) {
                std::cerr << "Error: Problem in line " << lineno << std::endl;
                return 1;
            }
            T = makeScaling(sx, sy, sz);
        } else if (kind == 'r') {
            double rx, ry, rz, angle;
            if (!(iss >> rx >> ry >> rz >> angle)) {
                std::cerr << "Error: Problem in line " << lineno << std::endl;
                return 1;
            }
            T = makeRotation(rx, ry, rz, angle);
        } else {
            std::cerr << "Warning: unknown transform type '" << kind << "' on line " << lineno << std::endl;
            continue;
        }

        // Left-multiply
        M = T * M;
    }

    Matrix4d Minv = M.inverse();

    std::cout << Minv << std::endl;

    return 0;
}
