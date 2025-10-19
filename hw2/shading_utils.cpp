#include "shading_utils.h"
#include "io_utils.h"
#include "transform_utils.h"
#include "raster_utils.h"

#include <cmath>
#include <Eigen/Dense>

using Eigen::Vector3d;


Vector3d lighting (const Vector3d& P, const Vector3d& n_in, const ObjectInstance& mat, const std::vector<Light>& lights, Vector3d e) {
    // Returns RGB coloring for a single point using diffuse, specular, and ambient lighting

    Eigen::Vector3d diff_sum = Vector3d::Zero();
    Eigen::Vector3d spec_sum = Vector3d::Zero();

    Vector3d n = n_in;
    if (n.squaredNorm() > 0) n.normalize();
    Vector3d e_dir = (e - P).normalized();

    for (const auto& lt : lights) {
        Vector3d l_p(lt.x, lt.y, lt.z);
        Vector3d l_c(lt.r, lt.g, lt.b);
        Vector3d l_dir = (l_p - P);
        double d = l_dir.norm();
        if (d > 0.0) l_dir /= d;
        double atten = 1.0 / (1.0 + lt.atten * d * d);

        diff_sum += atten * l_c * std::max(0.0, n.dot(l_dir));
        spec_sum += atten * l_c * std::pow(std::max(0.0, (e_dir + l_dir).normalized().dot(n)), mat.shininess);
    }

    Vector3d col = mat.ambient.array() + diff_sum.array() * mat.diffuse.array() + spec_sum.array() * mat.specular.array();
    return col.array().min(1.0);
}

bool is_backface(const std::vector<Vertex>& verts) {
    Vector3d v1 = as_vec3(verts[0]);
    Vector3d v2 = as_vec3(verts[1]);
    Vector3d v3 = as_vec3(verts[2]);

    return ((v3 - v2).cross(v1 - v2)[2] < 0);
}

void draw_wireframe(Image& img, Scene& scene) {
    world_to_view(scene);
    view_to_ndc(scene);
    ndc_to_screen(img, scene);

    for (const auto& obj_inst: scene.scene_objects){
        Object obj = obj_inst.obj;
        for (const auto& face: obj.faces){
            int x1 = static_cast<int>(std::lround(obj.vertices[face.v1].x));
            int y1 = static_cast<int>(std::lround(obj.vertices[face.v1].y));
            int x2 = static_cast<int>(std::lround(obj.vertices[face.v2].x));
            int y2 = static_cast<int>(std::lround(obj.vertices[face.v2].y));
            int x3 = static_cast<int>(std::lround(obj.vertices[face.v3].x));
            int y3 = static_cast<int>(std::lround(obj.vertices[face.v3].y));

            draw_line(x1, y1, x2, y2, 255, 255, 255, img);
            draw_line(x2, y2, x3, y3, 255, 255, 255, img);
            draw_line(x3, y3, x1, y1, 255, 255, 255, img);
        }
    }
}

void shade_by_mode(Image& img, Scene& scene, size_t mode) {
    if (mode == 3) {
        draw_wireframe(img, scene);
        return;
    }
    
    world_to_view(scene);

    for (auto& obj_inst : scene.scene_objects) {
        for (auto& face : obj_inst.obj.faces) {
            Object& obj = obj_inst.obj;

            std::vector<Vertex> verts = {obj.vertices[face.v1], obj.vertices[face.v2], obj.vertices[face.v3]};
            view_to_ndc(verts, scene);
            if (is_backface(verts)) {continue;}

            Vector3d v1 = as_vec3(obj.vertices[face.v1]);
            Vector3d n1 = as_vec3(obj.normals[face.vn1]);
            Vector3d v2 = as_vec3(obj.vertices[face.v2]);
            Vector3d n2 = as_vec3(obj.normals[face.vn2]);
            Vector3d v3 = as_vec3(obj.vertices[face.v3]);
            Vector3d n3 = as_vec3(obj.normals[face.vn3]);

            if (mode == 0) {
                // Gouraud
                Vector3d col1 = lighting(v1, n1, obj_inst, scene.lights);
                Vector3d col2 = lighting(v2, n2, obj_inst, scene.lights);
                Vector3d col3 = lighting(v3, n3, obj_inst, scene.lights);
                raster_triangle_gouraud(verts, img, col1, col2, col3);
            } else if (mode == 1) {
                // Phong
                raster_triangle_phong(verts, img, v1, v2, v3, n1, n2, n3, scene, obj_inst);
            } else {
                // Flat (default)
                Vector3d v_avg = (v1 + v2 + v3) / 3.0;
                Vector3d n_avg = (n1 + n2 + n3) / 3.0;
                Vector3d col = lighting(v_avg, n_avg, obj_inst, scene.lights);
                raster_triangle_flat(verts, img, col);
            }
            
        }
    }
}