#include "scene_types.h"
#include "io_utils.h"
#include "transform_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <cmath>


std::string parse_parent_path(const std::string& path) {
    std::size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return "";
    return path.substr(0, pos);
}

size_t parse_size_t(const char* str) {
    return static_cast<size_t>(std::stoul(str));
}

inline void put_pixel(int x,int y, uint8_t r,uint8_t g,uint8_t b,
                         int W,int H, std::vector<uint8_t>& img, float a=1.0) {
    if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) return;
    size_t py = (size_t)H - 1 - size_t(y);
    size_t idx = 3ull*(py*W + x);
    img[idx+0] = uint8_t((1.f - a)*img[idx+0] + a*r);
    img[idx+1] = uint8_t((1.f - a)*img[idx+1] + a*g);
    img[idx+2] = uint8_t((1.f - a)*img[idx+2] + a*b);
}

void draw_line(int x0,int y0,int x1,int y1,
               uint8_t r,uint8_t g,uint8_t b,
               int W,int H, std::vector<uint8_t>& img)
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
            put_pixel(y,x, r,g,b, W,H, img, w0);
            put_pixel(y+ystep,x, r,g,b, W,H, img, w1);
        } else {
            put_pixel(x,y, r,g,b, W,H, img, w0);
            put_pixel(x,y+ystep, r,g,b, W,H, img, w1);
        }

        errf += slope;
        while (errf >= 1.0f) {
            y += ystep;
            errf -= 1.0f;
        }
    }
}

std::vector<uint8_t> drawWireframe(std::vector<object> scene_objects, size_t xres, size_t yres){
    std::vector<uint8_t> img(xres*yres*3, 0); // background (black)
    for (const auto& obj: scene_objects){
        for (const auto& face: obj.faces){
            int x1 = static_cast<int>(std::lround(obj.vertices[face.v1].x));
            int y1 = static_cast<int>(std::lround(obj.vertices[face.v1].y));
            int x2 = static_cast<int>(std::lround(obj.vertices[face.v2].x));
            int y2 = static_cast<int>(std::lround(obj.vertices[face.v2].y));
            int x3 = static_cast<int>(std::lround(obj.vertices[face.v3].x));
            int y3 = static_cast<int>(std::lround(obj.vertices[face.v3].y));

            draw_line(x1, y1, x2, y2, 255, 255, 255, xres, yres, img);
            draw_line(x2, y2, x3, y3, 255, 255, 255, xres, yres, img);
            draw_line(x3, y3, x1, y1, 255, 255, 255, xres, yres, img);
        }
    }
    return img;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres]\n";
        return 1;
    }

    size_t xres = parse_size_t(argv[2]);
    size_t yres = parse_size_t(argv[3]);

    std::string parent_path = parse_parent_path(argv[1]);
    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cerr << "Could not open file: " << argv[1] << "\n";
        return 1;
    }

    // Returns camera parameters, and all objects *with transformations applied*
    ParseSceneFileResult scene = parseSceneFile(fin, parent_path);

    CameraTransforms cam_transforms = makeCameraMatrices(scene.camera_params);

    std::vector<object> scene_objects_ndc = applyCameraTransformsToObjects(scene.scene_objects.transformed_objects, cam_transforms);

    convertCoordsToPixels(scene_objects_ndc, xres, yres);

    std::vector<uint8_t> img = drawWireframe(scene_objects_ndc, xres, yres);

    writePPM(img, xres, yres);
}