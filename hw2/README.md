# HW1 Wireframe Renderer

This project builds a simple shaded renderer using gouraud or phong shading. Follow the steps below to compile and run it.

## Notes
* The code must be compiled as C++14 for compatibility with Eigen.
* If the scene_description_file.txt is passed as a path, then it is assumed that all object files in that scene description share the same parent path.

## Build
1. Open a terminal in the `hw2` directory.
2. Run `make` to compile the program:
   ```bash
   make
   ```
   This produces the executable `shaded_renderer`.

## Run
After building, execute the renderer with:
```bash
./wireframe_renderer [scene_description_file.txt] [xres] [yres] [mode]
```
**Mode options:**
| Mode | Shading Type     |
|------|------------------|
| `0`  | Gouraud shading  |
| `1`  | Phong shading    |
| `2`  | Flat shading     |
| `3`  | Wireframe render |

The program loads the default scene and writes the output image to `stdout`.
If you would like to store the output into an image, pipe stdout into a ppm file:
```bash
./wireframe_renderer [scene_description_file.txt] [xres] [yres] > image.ppm
```
You can then then view the file via
```bash
open image.ppm
```

## Clean
To remove the compiled executable, run:
```bash
make clean
```