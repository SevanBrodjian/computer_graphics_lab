#include "png_loader.h"

#include <png.h>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
struct FileCloser {
    void operator()(FILE* fp) const {
        if (fp) {
            std::fclose(fp);
        }
    }
};

void handle_png_error(png_structp png_ptr, png_const_charp msg) {
    throw std::runtime_error(std::string("libpng error: ") + (msg ? msg : ""));
}

void handle_png_warning(png_structp, png_const_charp) {
    // ignore warnings
}

} // namespace

PngData load_png_rgba(const std::string& filename) {
    std::unique_ptr<FILE, FileCloser> file(std::fopen(filename.c_str(), "rb"));
    if (!file) {
        throw std::runtime_error("Failed to open PNG file: " + filename);
    }

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        throw std::runtime_error("Failed to create png read struct");
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        throw std::runtime_error("Failed to create png info struct");
    }

    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        throw std::runtime_error("Failed to create png end info struct");
    }

    try {
        png_set_error_fn(png_ptr, nullptr, handle_png_error, handle_png_warning);
        png_init_io(png_ptr, file.get());
        png_read_info(png_ptr, info_ptr);

        png_uint_32 width = 0, height = 0;
        int bit_depth = 0, color_type = 0;
        png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type, nullptr, nullptr, nullptr);

        if (bit_depth == 16) {
            png_set_strip_16(png_ptr);
        }
        if (color_type == PNG_COLOR_TYPE_PALETTE) {
            png_set_palette_to_rgb(png_ptr);
        }
        if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
            png_set_expand_gray_1_2_4_to_8(png_ptr);
        }
        if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
            png_set_tRNS_to_alpha(png_ptr);
        }
        if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_PALETTE) {
            png_set_filler(png_ptr, 0xFF, PNG_FILLER_AFTER);
        }
        if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
            png_set_gray_to_rgb(png_ptr);
        }

        png_read_update_info(png_ptr, info_ptr);

        png_size_t row_bytes = png_get_rowbytes(png_ptr, info_ptr);
        std::vector<unsigned char> pixels;
        pixels.resize(static_cast<std::size_t>(row_bytes) * height);

        std::vector<png_bytep> row_pointers(height);
        for (png_uint_32 y = 0; y < height; ++y) {
            row_pointers[y] = pixels.data() + (height - 1 - y) * row_bytes;
        }

        png_read_image(png_ptr, row_pointers.data());
        png_read_end(png_ptr, end_info);

        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);

        PngData data;
        data.width = static_cast<int>(width);
        data.height = static_cast<int>(height);
        data.channels = 4;
        data.pixels = std::move(pixels);
        return data;
    } catch (...) {
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        throw;
    }
}
