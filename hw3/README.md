# HW3 OpenGL Renderer

This project builds a simple shaded renderer using OpenGL. Follow the steps below to compile and run it.

## Notes
* The code must be compiled as C++14 for compatibility with Eigen.
* If the scene_description_file.txt is passed as a path, then it is assumed that all object files in that scene description share the same parent path.

## Build
1. Open a terminal in the `hw3` directory.
2. Run `mkdir build && cd build` to create a build directory and go into it
3. Run  `cmake ..` and `cmake --build .` to build the project. This produces the executable `opengl_renderer`.

## Run
After building, execute the renderer with:
```bash
./opengl_renderer [scene_description_file.txt] [xres] [yres]
```