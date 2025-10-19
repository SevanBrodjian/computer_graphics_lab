#include "scene_types.h"
#include "io_utils.h"
#include "transform_utils.h"
#include "raster_utils.h"
#include "shading_utils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>


Image make_blank_image(size_t xres, size_t yres, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0) {
    std::vector<uint8_t> img(xres * yres * 3);
    for (size_t i = 0; i < img.size(); i += 3) {
        img[i] = r;
        img[i + 1] = g;
        img[i + 2] = b;
    }
    std::vector<double> z_buf(xres * yres, std::numeric_limits<double>::infinity());
    return Image{std::move(img), std::move(z_buf), xres, yres};
}

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres] [mode]\n";
        return 1;
    }

    // Parse args
    size_t xres = parse_size_t(argv[2]);
    size_t yres = parse_size_t(argv[3]);
    size_t mode = parse_size_t(argv[4]);
    if (mode > 3) {
        std::cerr << "Invalid mode: " << mode << ". Must be either 0, 1, 2 (flat shading), or 3 (wireframe).\n";
        return 1;
    }

    // Load file
    std::string parent_path = parse_parent_path(argv[1]);
    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cerr << "Could not open file: " << argv[1] << "\n";
        return 1;
    }

    // Returns camera parameters, lighting, and objects in World Space
    Scene scene = parse_scene_file(fin, parent_path);
    
    Image img = make_blank_image(xres, yres);
    shade_by_mode(img, scene, mode);

    write_ppm(img);
}