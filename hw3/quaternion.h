#ifndef HW3_QUATERNION_H
#define HW3_QUATERNION_H

#include <array>
#include <cmath>

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion identity() { return Quaternion(); }

    static Quaternion from_axis_angle(double axis_x, double axis_y, double axis_z, double angle) {
        double half = angle * 0.5;
        double s = std::sin(half);
        return Quaternion(std::cos(half), axis_x * s, axis_y * s, axis_z * s);
    }

    static Quaternion from_unit_vectors(const std::array<double,3>& from,
                                        const std::array<double,3>& to) {
        double dot = from[0]*to[0] + from[1]*to[1] + from[2]*to[2];
        std::array<double,3> cross{
            from[1]*to[2] - from[2]*to[1],
            from[2]*to[0] - from[0]*to[2],
            from[0]*to[1] - from[1]*to[0]
        };
        Quaternion q(dot + 1.0, cross[0], cross[1], cross[2]);
        if (q.length_squared() < 1e-12) {
            // Vectors are nearly opposite; choose arbitrary orthogonal axis
            std::array<double,3> ortho = std::abs(from[0]) < 0.9 ?
                std::array<double,3>{0.0, -from[2], from[1]} :
                std::array<double,3>{-from[1], from[0], 0.0};
            double norm = std::sqrt(ortho[0]*ortho[0] + ortho[1]*ortho[1] + ortho[2]*ortho[2]);
            ortho[0] /= norm; ortho[1] /= norm; ortho[2] /= norm;
            return Quaternion(0.0, ortho[0], ortho[1], ortho[2]);
        }
        return q.normalized();
    }

    Quaternion normalized() const {
        double len = std::sqrt(length_squared());
        if (len == 0.0) return Quaternion();
        return Quaternion(w / len, x / len, y / len, z / len);
    }

    double length_squared() const {
        return w*w + x*x + y*y + z*z;
    }

    Quaternion operator*(const Quaternion& rhs) const {
        return Quaternion(
            w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z,
            w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y,
            w*rhs.y - x*rhs.z + y*rhs.w + z*rhs.x,
            w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w
        );
    }

    std::array<double,16> to_matrix() const {
        Quaternion n = normalized();
        double xx = n.x * n.x;
        double yy = n.y * n.y;
        double zz = n.z * n.z;
        double xy = n.x * n.y;
        double xz = n.x * n.z;
        double yz = n.y * n.z;
        double wx = n.w * n.x;
        double wy = n.w * n.y;
        double wz = n.w * n.z;

        double m00 = 1.0 - 2.0 * (yy + zz);
        double m01 = 2.0 * (xy - wz);
        double m02 = 2.0 * (xz + wy);
        double m10 = 2.0 * (xy + wz);
        double m11 = 1.0 - 2.0 * (xx + zz);
        double m12 = 2.0 * (yz - wx);
        double m20 = 2.0 * (xz - wy);
        double m21 = 2.0 * (yz + wx);
        double m22 = 1.0 - 2.0 * (xx + yy);

        return {
            m00, m10, m20, 0.0,
            m01, m11, m21, 0.0,
            m02, m12, m22, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
    }
};

#endif
