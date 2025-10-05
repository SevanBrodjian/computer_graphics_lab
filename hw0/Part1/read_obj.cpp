#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>

struct vertex {
    float x;
    float y;
    float z;
};

struct face {
    unsigned int v1;
    unsigned int v2;
    unsigned int v3;
};

struct object {
    std::string filename;
    std::vector<vertex> vertices;
    std::vector<face> faces;

    void print(){
        std::cout << filename << ":" << std::endl << std::endl;

        for (std::size_t i = 1; i < vertices.size(); ++i) std::cout << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << std::endl;
        for (const auto &f : faces) std::cout << "f " << f.v1 << " " << f.v2 << " " << f.v3 << std::endl;
    }
};

int main(int argc, char* argv[]) {
    std::vector<object> objects;

    for (int i = 1; i < argc; ++i) {
        std::string filename = argv[i];
        std::ifstream file(filename);
        if (!file) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            continue;
        }
        
        std::vector<vertex> vertices;
        vertices.push_back({0.0f, 0.0f, 0.0f});
        std::vector<face> faces;

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            if (line[0] == '#' || std::isspace(line[0])) continue;

            char type = line[0];
            std::istringstream iss(line.substr(1));

            if (type != 'v' && type != 'f') {
                throw std::runtime_error("Invalid format: must start with 'v' or 'f'");
            }

            if (type == 'v') {
                float x, y, z;
                if (!(iss >> x >> y >> z)) {
                    throw std::runtime_error("Invalid vertex format: expected 3 floats");
                }
                // Ensure there’s no extra junk after the numbers
                std::string leftover;
                if (iss >> leftover) {
                    throw std::runtime_error("Invalid vertex format: extra data found");
                }
                vertices.push_back({x, y, z});
            } 
            else if (type == 'f') {
                unsigned int a, b, c;
                if (!(iss >> a >> b >> c)) {
                    throw std::runtime_error("Invalid face format: expected 3 unsigned ints");
                }
                std::string leftover;
                if (iss >> leftover) {
                    throw std::runtime_error("Invalid face format: extra data found");
                }
                faces.push_back({a, b, c});
            }
        }

        file.close();
        objects.push_back({filename, vertices, faces});
    }

    for (std::size_t i = 0; i < objects.size(); ++i) {
        objects[i].print();
        if (i != objects.size() - 1) std::cout << std::endl;
    }

    return 0;
}