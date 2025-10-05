#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <cstdint>

struct color {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    void print() {
        std::cout << static_cast<int>(r) << " "
                << static_cast<int>(g) << " "
                << static_cast<int>(b) << std::endl;
    }
};

size_t parse_size_t(const char* str) {
    try {
        // std::stoul converts string to unsigned long
        std::string s(str);
        size_t pos;
        unsigned long val = std::stoul(s, &pos);

        // Make sure the whole string was numeric
        if (pos != s.size()) {
            throw std::invalid_argument("Invalid number: extra characters");
        }

        // Check for overflow if size_t is smaller than unsigned long
        if (val > static_cast<unsigned long>(std::numeric_limits<size_t>::max())) {
            throw std::out_of_range("Value too large for size_t");
        }

        return static_cast<size_t>(val);

    } catch (const std::invalid_argument&) {
        throw std::invalid_argument("Cannot convert to size_t: not a number");
    } catch (const std::out_of_range&) {
        throw std::out_of_range("Cannot convert to size_t: out of range");
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " xres yres" << std::endl;
        return 1;
    }

    size_t xres = parse_size_t(argv[1]);
    size_t yres = parse_size_t(argv[2]);

    // Header info for P3
    std::cout << "P3" <<std::endl;
    std::cout << xres << " " << yres << std::endl;
    std::cout << "255" << std::endl;

    color background_col = color({150, 30, 30}); // Deep red
    color circle_col = color({30, 150, 30}); // Deep green
    size_t radius = std::min(xres, yres) / 2 / 2;
    size_t r2 = radius * radius;
    int c_x = xres / 2; 
    int c_y = yres / 2;

    for(int i = 0; i < xres; ++i){
        for(int j = 0; j < yres; ++j){
            int xpos = i - c_x;
            int ypos = j - c_y;
            if (xpos*xpos + ypos*ypos <= r2)
                circle_col.print();
            else
                background_col.print();
        }
    }

    return 0;
}