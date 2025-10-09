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

    std::vector<vertex> scene_objects_vertices = convertCoordsToPixels(scene_objects_ndc, xres, yres);

    std::cout << "P3" <<std::endl;
    std::cout << xres << " " << yres << std::endl;
    std::cout << "255" << std::endl;
    color background_col{uint8_t(0), uint8_t(0),  uint8_t(0)};
    color draw_col      {uint8_t(255),  uint8_t(255), uint8_t(255)};

    for(size_t j = 0; j < yres; ++j){
        for(size_t i = 0; i < xres; ++i){
            bool anyVerts = false;
            for (const auto& vert : scene_objects_vertices) {
                size_t pixelX = static_cast<size_t>(std::round(vert.x));
                size_t pixelY = static_cast<size_t>(std::round(vert.y));
                if (pixelX == i && pixelY == j) {
                    draw_col.print();
                    anyVerts = true;
                }
                    
            }
            if (!anyVerts)
                background_col.print();
        }
    }
}