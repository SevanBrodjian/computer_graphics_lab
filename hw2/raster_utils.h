#ifndef RASTER_UTILS_H
#define RASTER_UTILS_H

#include "scene_types.h"

#include <cmath>
#include <vector>
#include <Eigen/Dense>

using Eigen::Vector3d;


void put_pixel(int x,int y, double z, uint8_t r,uint8_t g,uint8_t b,
                         Image& img, float a=1.0);

void draw_line(int x0,int y0,int x1,int y1,
               uint8_t r,uint8_t g,uint8_t b,
               Image& img);

void raster_triangle_flat(std::vector<Vertex>& verts, Image& img, Vector3d col);

void raster_triangle_gouraud(std::vector<Vertex>& verts, Image& img, Vector3d col1, Vector3d col2, Vector3d col3);

void raster_triangle_phong(std::vector<Vertex>& verts, Image& img, 
                            Vector3d v1, Vector3d v2, Vector3d v3, 
                            Vector3d n1, Vector3d n2, Vector3d n3, 
                            const Scene& scene, const ObjectInstance& obj_inst);

#endif