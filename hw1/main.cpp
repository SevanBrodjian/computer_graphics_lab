#include "io_utils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


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
    ParseSceneFileResult scene = parseSceneFile(fin, parent_path);

    CameraTransforms cam_transforms = makeCameraMatrices(scene.camera_params);
}