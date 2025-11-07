#ifndef HW4_PNG_LOADER_H
#define HW4_PNG_LOADER_H

#include <string>
#include <vector>

struct PngData {
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<unsigned char> pixels;
};

PngData load_png_rgba(const std::string& filename);

#endif
