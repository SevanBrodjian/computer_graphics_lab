#ifndef HW3_ARCBALL_H
#define HW3_ARCBALL_H

#include "quaternion.h"

#include <array>
#include <cmath>

class Arcball {
public:
    void set_window(int width, int height) {
        window_width_ = width;
        window_height_ = height;
    }

    void begin_drag(int x, int y) {
        dragging_ = true;
        start_vec_ = map_to_sphere(x, y);
        base_rotation_ = current_rotation_;
    }

    void update_drag(int x, int y) {
        if (!dragging_) return;
        auto current_vec = map_to_sphere(x, y);
        Quaternion delta = Quaternion::from_unit_vectors(start_vec_, current_vec);
        current_rotation_ = delta * base_rotation_;
    }

    void end_drag() {
        dragging_ = false;
    }

    Quaternion rotation() const { return current_rotation_; }

private:
    std::array<double,3> map_to_sphere(int x, int y) const {
        if (window_width_ == 0 || window_height_ == 0) {
            return {0.0, 0.0, 1.0};
        }
        double nx = (2.0 * x - window_width_) / window_width_;
        double ny = (window_height_ - 2.0 * y) / window_height_;
        double length = nx*nx + ny*ny;
        double nz;
        if (length > 1.0) {
            double norm = std::sqrt(length);
            nx /= norm;
            ny /= norm;
            nz = 0.0;
        } else {
            nz = std::sqrt(1.0 - length);
        }
        return {nx, ny, nz};
    }

    int window_width_{1};
    int window_height_{1};
    bool dragging_{false};
    std::array<double,3> start_vec_{0.0, 0.0, 1.0};
    Quaternion base_rotation_ = Quaternion::identity();
    Quaternion current_rotation_ = Quaternion::identity();
};

#endif
