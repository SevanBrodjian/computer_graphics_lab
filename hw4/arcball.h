#ifndef HW3_ARCBALL_H
#define HW3_ARCBALL_H

#include "quaternion.h"

#include <algorithm>
#include <array>
#include <cmath>

class Arcball {
public:
    void set_window(int width, int height) {
        window_width_ = std::max(width, 1);
        window_height_ = std::max(height, 1);
    }

    void set_viewport(int vx, int vy, int vw, int vh) {
        viewport_x_ = vx;
        viewport_y_ = vy;
        viewport_width_ = std::max(vw, 1);
        viewport_height_ = std::max(vh, 1);
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
        if (viewport_width_ <= 0 || viewport_height_ <= 0) {
            return {0.0, 0.0, 1.0};
        }

        // Translate the mouse position into viewport space and normalize to [-1, 1]
        double vx = static_cast<double>(x - viewport_x_);
        double vy = static_cast<double>(viewport_height_ - (y - viewport_y_));
        double nx = (2.0 * vx - viewport_width_) / viewport_width_;
        double ny = (2.0 * vy - viewport_height_) / viewport_height_;

        double length = nx * nx + ny * ny;
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
    int viewport_x_{0};
    int viewport_y_{0};
    int viewport_width_{1};
    int viewport_height_{1};
    bool dragging_{false};
    std::array<double,3> start_vec_{0.0, 0.0, 1.0};
    Quaternion base_rotation_ = Quaternion::identity();
    Quaternion current_rotation_ = Quaternion::identity();
};

#endif
