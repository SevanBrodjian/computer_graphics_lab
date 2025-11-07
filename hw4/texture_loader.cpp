#include "texture_loader.h"

#include <png.h>

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {
struct PngReadGuard {
    png_structp png_ptr{nullptr};
    png_infop info_ptr{nullptr};
    png_infop end_info{nullptr};
    FILE* file{nullptr};

    ~PngReadGuard() {
        if (png_ptr || info_ptr || end_info) {
            png_destroy_read_struct(png_ptr ? &png_ptr : nullptr,
                                    info_ptr ? &info_ptr : nullptr,
                                    end_info ? &end_info : nullptr);
        }
        if (file) {
            std::fclose(file);
        }
    }
};
}

GLuint load_png_texture(const std::string& filename) {
    PngReadGuard guard;
    guard.file = std::fopen(filename.c_str(), "rb");
    if (!guard.file) {
        throw std::runtime_error("Failed to open PNG file: " + filename);
    }

    png_byte header[8];
    if (std::fread(header, 1, sizeof(header), guard.file) != sizeof(header)) {
        throw std::runtime_error("Failed to read PNG header: " + filename);
    }
    if (png_sig_cmp(header, 0, sizeof(header)) != 0) {
        throw std::runtime_error("File is not a valid PNG image: " + filename);
    }

    guard.png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!guard.png_ptr) {
        throw std::runtime_error("png_create_read_struct failed for: " + filename);
    }

    guard.info_ptr = png_create_info_struct(guard.png_ptr);
    guard.end_info = png_create_info_struct(guard.png_ptr);
    if (!guard.info_ptr || !guard.end_info) {
        throw std::runtime_error("png_create_info_struct failed for: " + filename);
    }

    if (setjmp(png_jmpbuf(guard.png_ptr))) {
        throw std::runtime_error("libpng error while reading: " + filename);
    }

    png_init_io(guard.png_ptr, guard.file);
    png_set_sig_bytes(guard.png_ptr, sizeof(header));
    png_read_info(guard.png_ptr, guard.info_ptr);

    png_uint_32 width = 0;
    png_uint_32 height = 0;
    int bit_depth = 0;
    int color_type = 0;
    png_get_IHDR(guard.png_ptr, guard.info_ptr, &width, &height, &bit_depth, &color_type, nullptr, nullptr, nullptr);

    if (bit_depth == 16) {
        png_set_strip_16(guard.png_ptr);
    }
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(guard.png_ptr);
    }
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
        png_set_expand_gray_1_2_4_to_8(guard.png_ptr);
    }
    if (png_get_valid(guard.png_ptr, guard.info_ptr, PNG_INFO_tRNS)) {
        png_set_tRNS_to_alpha(guard.png_ptr);
    }
    if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY ||
        color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_filler(guard.png_ptr, 0xFF, PNG_FILLER_AFTER);
    }
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(guard.png_ptr);
    }

    png_read_update_info(guard.png_ptr, guard.info_ptr);

    png_size_t rowbytes = png_get_rowbytes(guard.png_ptr, guard.info_ptr);
    std::vector<png_byte> image_data(rowbytes * height);
    std::vector<png_bytep> row_pointers(height);
    for (png_uint_32 y = 0; y < height; ++y) {
        row_pointers[y] = image_data.data() + y * rowbytes;
    }

    png_read_image(guard.png_ptr, row_pointers.data());
    png_read_end(guard.png_ptr, guard.end_info);

    // Flip the image vertically so textures appear right-side up
    std::vector<png_byte> flipped(rowbytes * height);
    for (png_uint_32 y = 0; y < height; ++y) {
        std::memcpy(flipped.data() + (height - 1 - y) * rowbytes,
                    image_data.data() + y * rowbytes,
                    rowbytes);
    }

    GLuint texture = 0;
    glGenTextures(1, &texture);
    if (texture == 0) {
        throw std::runtime_error("glGenTextures failed while loading: " + filename);
    }

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, static_cast<GLsizei>(width), static_cast<GLsizei>(height), 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, flipped.data());
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}
