# HW6 Simulation and Animation

Follow the steps below to compile and run each part of this homework.

## Notes
* Makefiles and a few lines of code were adjusted so everything builds and runs on my MacBook. I tried to keep the setup compatible with Linux, but I have not tested it there yet.
* All projects use C++14 with Eigen and OpenGL/GLUT. 
* The bunny interpolation produces obj files that match the provided targets up to 5 decimal places, but 'diff' still shows tiny numerical differences that I can't eliminate.

## Part 1: Spring Pendulums and Elasticity
### Single Spring Pendulum
1. Open a terminal in `hw6/Part1/Single_Spring_Pendulum`.
2. Run `make` to compile the demo:
```bash
make
```
This produces the executable `single_pendulum`.
3. Launch the visualization with intial conditions (x(0), y(0)):
```bash
./single_pendulum [xres] [yres] [x(0)] [y(0)]
```
4. Clean the build outputs with:
```bash
make clean
```

### Double Spring Pendulum
1. Open a terminal in `hw6/Part1/Double_Spring_Pendulum`.
2. Build the program:
```bash
make
```
This creates the executable `double_pendulum`.
3. Run the demo with initial conditions (x1(0), y1(0)) for the first pendulum and (x2(0), y2(0)) for the second:
```bash
./double_pendulum [xres] [yres] [x1(0)] [y1(0)] [x2(0)] [y2(0)]
```
4. Remove compiled files with:
```bash
make clean
```

### Elasticity Demo
1. Navigate to `hw6/Part1/Elasticity`.
2. Compile with:
```bash
make
```
The build generates the executable `simulate`.
3. Run the simulation on the provided mesh:
```bash
./simulate man.obj [xres] [yres]
```
You can replace `man.obj` with another OBJ path to test a different mesh.
4. Clean up when finished:
```bash
make clean
```

## Part 2: Keyframe Animation
### Bunny Frame Interpolation
1. Change into `hw6/Part2/Bunny_Frames`.
2. Build the interpolator:
```bash
make
```
This produces the executable `bunny_interpolate`.
3. Generate in-between frames for the bunny keyframes:
```bash
./bunny_interpolate
```
Interpolated OBJ files are written into `generated_frames/`.
4. Remove outputs with:
```bash
make clean
```

### I-Bar Keyframe Viewer
1. Go to `hw6/Part2/I_Bar`.
2. Compile the viewer:
```bash
make
```
The executable `keyframe` is created.
3. Run the viewer with an animation script (defaults to `test.script` if none is provided):
```bash
./keyframe [script_file]
```
4. Press any key (except 'q' or 'Esc') to advance the animation by one frame, or press 'q' or 'Esc' to quit the program.
5. Clean the directory with:
```bash
make clean
```