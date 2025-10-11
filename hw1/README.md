# HW1 Wireframe Renderer

This project builds a simple wireframe renderer. Follow the steps below to compile and run it.

## Notes
* The code must be compiled as C++14 for compatibility with Eigen.
* If the scene_description_file.txt is passed as a path, then it is assumed that all object files in that scene description share the same parent path.
* It seems in the given targets lines are not rasterized if one of the points is offscreen, however it was actually simpler (and cleaner) to allow these lines to be drawn up to the boundaries. If it's required that the outputs are identical to the targets, please see the note on main.cpp and uncomment lines 39-40.

## Build
1. Open a terminal in the `hw1` directory.
2. Run `make` to compile the program:
   ```bash
   make
   ```
   This produces the executable `wireframe_renderer`.

## Run
After building, execute the renderer with:
```bash
./wireframe_renderer [scene_description_file.txt] [xres] [yres]
```
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

## Extending Bresenham's
In the default case, Bresenham's algorithm operates with a slope between 0 and 1. However, there is symmetry across all the octants which allows us to easily apply this algorithm to all the other possible lines in 360 degrees. We handle the symmetries in three steps:
1. If the slope is greater than 1 (dy > dx), then we simply swap our axes and run the same algorithm, exploiting the symmetry between the first two octants. Later, we must be careful to put our pixels in the right place by undoing this swap when drawing, otherwise the algorithm is unchanged. This allows us to cover lines anywhere in the first quadrant.
2. Next, we can easily handle the third quadrant by ensuring that lines are always drawn from left to right on the x axis (after swapping in step 1 as needed). This exploits the symmetry that lines where dx and dy are both negative are equivalent if we swap endpoints and assume +dx and +dy. 
3. Finally, to handle the remaining quadrants we must simply determine whether our y step is going up or down. This is determined by simply checking te sign of dy. Since this is done after our swappings from steps 1 and 2 this handles both quadrant 2 and 4.