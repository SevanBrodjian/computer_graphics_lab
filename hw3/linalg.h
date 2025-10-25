#ifndef HW3_LINALG_H
#define HW3_LINALG_H

#include <array>
#include <cmath>
#include <algorithm>

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    double& operator[](std::size_t idx) {
        return idx == 0 ? x : (idx == 1 ? y : z);
    }

    const double& operator[](std::size_t idx) const {
        return idx == 0 ? x : (idx == 1 ? y : z);
    }

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z; return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this;
    }

    Vec3& operator*=(double s) {
        x *= s; y *= s; z *= s; return *this;
    }

    Vec3& operator/=(double s) {
        x /= s; y /= s; z /= s; return *this;
    }

    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3 normalized() const {
        double n = norm();
        if (n == 0.0) {
            return Vec3{};
        }
        return Vec3{x / n, y / n, z / n};
    }

    static Vec3 zero() { return Vec3{}; }
};

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
    lhs += rhs;
    return lhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Vec3 operator*(Vec3 lhs, double s) {
    lhs *= s;
    return lhs;
}

inline Vec3 operator*(double s, Vec3 rhs) {
    rhs *= s;
    return rhs;
}

inline Vec3 operator/(Vec3 lhs, double s) {
    lhs /= s;
    return lhs;
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

struct Mat3 {
    std::array<double, 9> m{}; // column-major

    static Mat3 identity() {
        Mat3 I;
        I.m = {1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0};
        return I;
    }

    double& operator()(int row, int col) {
        return m[col * 3 + row];
    }

    const double& operator()(int row, int col) const {
        return m[col * 3 + row];
    }

    Mat3 transpose() const {
        Mat3 t;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                t(r, c) = (*this)(c, r);
            }
        }
        return t;
    }

    double determinant() const {
        return (*this)(0,0) * ((*this)(1,1) * (*this)(2,2) - (*this)(1,2) * (*this)(2,1))
             - (*this)(0,1) * ((*this)(1,0) * (*this)(2,2) - (*this)(1,2) * (*this)(2,0))
             + (*this)(0,2) * ((*this)(1,0) * (*this)(2,1) - (*this)(1,1) * (*this)(2,0));
    }

    Mat3 inverse() const {
        double det = determinant();
        if (std::abs(det) < 1e-15) {
            return Mat3::identity();
        }
        double inv_det = 1.0 / det;
        Mat3 inv;
        inv(0,0) = ((*this)(1,1) * (*this)(2,2) - (*this)(1,2) * (*this)(2,1)) * inv_det;
        inv(0,1) = ((*this)(0,2) * (*this)(2,1) - (*this)(0,1) * (*this)(2,2)) * inv_det;
        inv(0,2) = ((*this)(0,1) * (*this)(1,2) - (*this)(0,2) * (*this)(1,1)) * inv_det;

        inv(1,0) = ((*this)(1,2) * (*this)(2,0) - (*this)(1,0) * (*this)(2,2)) * inv_det;
        inv(1,1) = ((*this)(0,0) * (*this)(2,2) - (*this)(0,2) * (*this)(2,0)) * inv_det;
        inv(1,2) = ((*this)(0,2) * (*this)(1,0) - (*this)(0,0) * (*this)(1,2)) * inv_det;

        inv(2,0) = ((*this)(1,0) * (*this)(2,1) - (*this)(1,1) * (*this)(2,0)) * inv_det;
        inv(2,1) = ((*this)(0,1) * (*this)(2,0) - (*this)(0,0) * (*this)(2,1)) * inv_det;
        inv(2,2) = ((*this)(0,0) * (*this)(1,1) - (*this)(0,1) * (*this)(1,0)) * inv_det;
        return inv;
    }
};

inline Vec3 operator*(const Mat3& A, const Vec3& v) {
    return Vec3{
        A(0,0) * v.x + A(0,1) * v.y + A(0,2) * v.z,
        A(1,0) * v.x + A(1,1) * v.y + A(1,2) * v.z,
        A(2,0) * v.x + A(2,1) * v.y + A(2,2) * v.z
    };
}

struct Mat4 {
    std::array<double, 16> m{}; // column-major

    static Mat4 identity() {
        Mat4 I;
        I.m = {1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0};
        return I;
    }

    double& operator()(int row, int col) {
        return m[col * 4 + row];
    }

    const double& operator()(int row, int col) const {
        return m[col * 4 + row];
    }

    Mat4 operator*(const Mat4& rhs) const {
        Mat4 result;
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                double sum = 0.0;
                for (int k = 0; k < 4; ++k) {
                    sum += (*this)(r, k) * rhs(k, c);
                }
                result(r, c) = sum;
            }
        }
        return result;
    }

    Mat4 transpose() const {
        Mat4 t;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                t(r, c) = (*this)(c, r);
            }
        }
        return t;
    }

    Mat3 top_left_3x3() const {
        Mat3 A;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                A(r, c) = (*this)(r, c);
            }
        }
        return A;
    }

    static Mat4 translation(double tx, double ty, double tz) {
        Mat4 T = Mat4::identity();
        T(0, 3) = tx;
        T(1, 3) = ty;
        T(2, 3) = tz;
        return T;
    }

    static Mat4 scaling(double sx, double sy, double sz) {
        Mat4 S = Mat4::identity();
        S(0, 0) = sx;
        S(1, 1) = sy;
        S(2, 2) = sz;
        return S;
    }

    static Mat4 rotation(double axis_x, double axis_y, double axis_z, double angle) {
        Vec3 axis{axis_x, axis_y, axis_z};
        double len = axis.norm();
        if (len == 0.0) {
            return Mat4::identity();
        }
        axis /= len;
        double c = std::cos(angle);
        double s = std::sin(angle);
        double t = 1.0 - c;

        Mat4 R = Mat4::identity();
        R(0,0) = t * axis.x * axis.x + c;
        R(0,1) = t * axis.x * axis.y - s * axis.z;
        R(0,2) = t * axis.x * axis.z + s * axis.y;

        R(1,0) = t * axis.x * axis.y + s * axis.z;
        R(1,1) = t * axis.y * axis.y + c;
        R(1,2) = t * axis.y * axis.z - s * axis.x;

        R(2,0) = t * axis.x * axis.z - s * axis.y;
        R(2,1) = t * axis.y * axis.z + s * axis.x;
        R(2,2) = t * axis.z * axis.z + c;
        return R;
    }

    const double* data() const { return m.data(); }
    double* data() { return m.data(); }
};

inline Vec3 transform_point(const Mat4& M, const Vec3& v) {
    double x = M(0,0) * v.x + M(0,1) * v.y + M(0,2) * v.z + M(0,3);
    double y = M(1,0) * v.x + M(1,1) * v.y + M(1,2) * v.z + M(1,3);
    double z = M(2,0) * v.x + M(2,1) * v.y + M(2,2) * v.z + M(2,3);
    double w = M(3,0) * v.x + M(3,1) * v.y + M(3,2) * v.z + M(3,3);
    if (w != 0.0 && w != 1.0) {
        double inv_w = 1.0 / w;
        x *= inv_w;
        y *= inv_w;
        z *= inv_w;
    }
    return Vec3{x, y, z};
}

inline Vec3 transform_vector(const Mat4& M, const Vec3& v) {
    return Vec3{
        M(0,0) * v.x + M(0,1) * v.y + M(0,2) * v.z,
        M(1,0) * v.x + M(1,1) * v.y + M(1,2) * v.z,
        M(2,0) * v.x + M(2,1) * v.y + M(2,2) * v.z
    };
}

inline Mat4 perspective(double n, double f, double l, double r, double b, double t) {
    Mat4 P{};
    P(0,0) = (2.0 * n) / (r - l);
    P(0,1) = 0.0;
    P(0,2) = (r + l) / (r - l);
    P(0,3) = 0.0;

    P(1,0) = 0.0;
    P(1,1) = (2.0 * n) / (t - b);
    P(1,2) = (t + b) / (t - b);
    P(1,3) = 0.0;

    P(2,0) = 0.0;
    P(2,1) = 0.0;
    P(2,2) = -(f + n) / (f - n);
    P(2,3) = -(2.0 * f * n) / (f - n);

    P(3,0) = 0.0;
    P(3,1) = 0.0;
    P(3,2) = -1.0;
    P(3,3) = 0.0;
    return P;
}

#endif
