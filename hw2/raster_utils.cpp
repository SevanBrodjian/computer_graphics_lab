#include "raster_utils.h"
#include "scene_types.h"
#include "transform_utils.h"
#include "shading_utils.h"

#include <cmath>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Eigen::Vector3d;


inline void put_pixel(int x, int y, double z, uint8_t r, uint8_t g, uint8_t b,
                         Image& img, float a) {
    size_t W = img.xres;
    size_t H = img.yres;
    if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) return;
    if (z < -1 || z > 1) return;
    size_t py = (size_t)H - 1 - size_t(y);
    size_t buf_idx = (py*W + x);
    size_t idx = 3ull*buf_idx;
    if (z > img.z_buf[buf_idx]) return;
    img.img[idx+0] = uint8_t((1.f - a)*img.img[idx+0] + a*r);
    img.img[idx+1] = uint8_t((1.f - a)*img.img[idx+1] + a*g);
    img.img[idx+2] = uint8_t((1.f - a)*img.img[idx+2] + a*b);
    img.z_buf[buf_idx] = z;
}

void draw_line(int x0,int y0,int x1,int y1,
               uint8_t r,uint8_t g,uint8_t b,
               Image& img)
{
    // NOTE: Uncomment to avoid drawing lines to points that are offscreen
    // if ((unsigned)x0 > (unsigned)W || (unsigned)y0 > (unsigned)H ||
    //         (unsigned)x1 > (unsigned)W || (unsigned)y1 > (unsigned)H) return;
    
    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep) { std::swap(x0, y0); std::swap(x1, y1); }
    if (x0 > x1) { std::swap(x0, x1); std::swap(y0, y1); }

    int dx = x1 - x0;
    int dy_abs = std::abs(y1 - y0);
    int ystep = (y0 < y1) ? 1 : -1;

    // fractional error in [0,1), how far the true line is toward the next y
    float errf = 0.0f;
    float slope = dx ? (float)dy_abs / (float)dx : 0.0f;

    int y = y0;
    for (int x = x0; x <= x1; ++x) {
        // Nearer pixel gets weight 1-errf, the other gets errf
        float w0 = 1.0f - errf; // current y
        float w1 = errf; // neighbor at y + ystep

        if (steep) {
            put_pixel(y,x,0.0, r,g,b, img, w0);
            put_pixel(y+ystep,x,0.0, r,g,b, img, w1);
        } else {
            put_pixel(x,y,0.0, r,g,b, img, w0);
            put_pixel(x,y+ystep,0.0, r,g,b, img, w1);
        }

        errf += slope;
        while (errf >= 1.0f) {
            y += ystep;
            errf -= 1.0f;
        }
    }
}

struct ComputeABGResult {
    double alpha, beta, gamma;
};

ComputeABGResult compute_abg(int x_a, int x_b, int x_c, int y_a, int y_b, int y_c, int x, int y){
    auto f = [] (int xi, int xj, int yi, int yj, int xp, int yp) { 
        return (yi - yj) * xp + (xj - xi) * yp + xi * yj - xj * yi; 
    };
    
    double alpha_num = f(x_b, x_c, y_b, y_c, x, y);
    double alpha_denom = f(x_b, x_c, y_b, y_c, x_a, y_a);
    double alpha = alpha_num / alpha_denom;

    double beta_num = f(x_a, x_c, y_a, y_c, x, y);
    double beta_denom = f(x_a, x_c, y_a, y_c, x_b, y_b);
    double beta = beta_num / beta_denom;

    double gamma_num = f(x_a, x_b, y_a, y_b, x, y);
    double gamma_denom = f(x_a, x_b, y_a, y_b, x_c, y_c);
    double gamma = gamma_num / gamma_denom;

    return ComputeABGResult({alpha, beta, gamma});
}

void raster_triangle_flat(std::vector<Vertex>& verts, Image& img, Vector3d col) {
    ndc_to_screen(img, verts);
    uint8_t r = static_cast<uint8_t>(col[0] * 255);
    uint8_t g = static_cast<uint8_t>(col[1] * 255);
    uint8_t b = static_cast<uint8_t>(col[2] * 255);

    int x_a = static_cast<int>(std::lround(verts[0].x));
    int y_a = static_cast<int>(std::lround(verts[0].y));
    double z_a = verts[0].z;
    int x_b = static_cast<int>(std::lround(verts[1].x));
    int y_b = static_cast<int>(std::lround(verts[1].y));
    double z_b = verts[1].z;
    int x_c = static_cast<int>(std::lround(verts[2].x));
    int y_c = static_cast<int>(std::lround(verts[2].y));
    double z_c = verts[2].z;

    int x_min = static_cast<int>(std::min({x_a, x_b, x_c}));
    int x_max = static_cast<int>(std::max({x_a, x_b, x_c}));
    int y_min = static_cast<int>(std::min({y_a, y_b, y_c}));
    int y_max = static_cast<int>(std::max({y_a, y_b, y_c}));

    for (auto x = x_min; x <= x_max; ++x) {
        for (auto y = y_min; y <= y_max; ++y) {
            ComputeABGResult abg = compute_abg(x_a, x_b, x_c, y_a, y_b, y_c, x, y);
            double alpha = abg.alpha;
            double beta = abg.beta;
            double gamma = abg.gamma;

            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1){
                double z = alpha * z_a + beta * z_b + gamma * z_c;
                put_pixel(x, y, z, r, g, b, img);
            }
        }
    }
}

void raster_triangle_gouraud(std::vector<Vertex>& verts, Image& img, Vector3d col1, Vector3d col2, Vector3d col3) {
    ndc_to_screen(img, verts);

    int x_a = static_cast<int>(std::lround(verts[0].x));
    int y_a = static_cast<int>(std::lround(verts[0].y));
    double z_a = verts[0].z;
    int x_b = static_cast<int>(std::lround(verts[1].x));
    int y_b = static_cast<int>(std::lround(verts[1].y));
    double z_b = verts[1].z;
    int x_c = static_cast<int>(std::lround(verts[2].x));
    int y_c = static_cast<int>(std::lround(verts[2].y));
    double z_c = verts[2].z;

    int x_min = static_cast<int>(std::min({x_a, x_b, x_c}));
    int x_max = static_cast<int>(std::max({x_a, x_b, x_c}));
    int y_min = static_cast<int>(std::min({y_a, y_b, y_c}));
    int y_max = static_cast<int>(std::max({y_a, y_b, y_c}));

    for (auto x = x_min; x <= x_max; ++x) {
        for (auto y = y_min; y <= y_max; ++y) {
            ComputeABGResult abg = compute_abg(x_a, x_b, x_c, y_a, y_b, y_c, x, y);
            double alpha = abg.alpha;
            double beta = abg.beta;
            double gamma = abg.gamma;

            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1){
                double z = alpha * z_a + beta * z_b + gamma * z_c;
                Vector3d col = alpha * col1 + beta * col2 + gamma * col3;
                uint8_t r = static_cast<uint8_t>(col[0] * 255);
                uint8_t g = static_cast<uint8_t>(col[1] * 255);
                uint8_t b = static_cast<uint8_t>(col[2] * 255);
                put_pixel(x, y, z, r, g, b, img);
            }
        }
    }
}

void raster_triangle_phong(std::vector<Vertex>& verts, Image& img, 
                            Vector3d v1, Vector3d v2, Vector3d v3, 
                            Vector3d n1, Vector3d n2, Vector3d n3, 
                            const Scene& scene, const ObjectInstance& obj_inst) {
    ndc_to_screen(img, verts);

    int x_a = static_cast<int>(std::lround(verts[0].x));
    int y_a = static_cast<int>(std::lround(verts[0].y));
    double z_a = verts[0].z;
    int x_b = static_cast<int>(std::lround(verts[1].x));
    int y_b = static_cast<int>(std::lround(verts[1].y));
    double z_b = verts[1].z;
    int x_c = static_cast<int>(std::lround(verts[2].x));
    int y_c = static_cast<int>(std::lround(verts[2].y));
    double z_c = verts[2].z;

    int x_min = static_cast<int>(std::min({x_a, x_b, x_c}));
    int x_max = static_cast<int>(std::max({x_a, x_b, x_c}));
    int y_min = static_cast<int>(std::min({y_a, y_b, y_c}));
    int y_max = static_cast<int>(std::max({y_a, y_b, y_c}));

    for (auto x = x_min; x <= x_max; ++x) {
        for (auto y = y_min; y <= y_max; ++y) {
            ComputeABGResult abg = compute_abg(x_a, x_b, x_c, y_a, y_b, y_c, x, y);
            double alpha = abg.alpha;
            double beta = abg.beta;
            double gamma = abg.gamma;

            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1){
                double z = alpha * z_a + beta * z_b + gamma * z_c;
                Vector3d v = alpha * v1 + beta * v2 + gamma * v3;
                Vector3d n = alpha * n1 + beta * n2 + gamma * n3;
                Vector3d col = lighting(v, n, obj_inst, scene.lights);
                uint8_t r = static_cast<uint8_t>(col[0] * 255);
                uint8_t g = static_cast<uint8_t>(col[1] * 255);
                uint8_t b = static_cast<uint8_t>(col[2] * 255);
                put_pixel(x, y, z, r, g, b, img);
            }
        }
    }
}