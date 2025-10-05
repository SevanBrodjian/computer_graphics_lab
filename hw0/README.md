# HW0 Build & Run Guide

This quick-reference README summarizes how to compile and execute each part of the HW0 assignment.

## Part 1 – OBJ Reader
1. `cd hw0/Part1`
2. Compile with `make` (produces the `read_obj` executable).
3. Run against one or more OBJ files, e.g. `./read_obj tetrahedron.obj`.
   * Pass multiple paths to print each object in sequence.
4. Optional cleanup: `make clean`.

## Part 2 – Inverse Transform Calculator
1. `cd hw0/Part2`
2. Compile with `make` (produces the `inverse_transform` executable).
3. Provide a transform list, e.g. `./inverse_transform transform_data1.txt`.
   * The program prints the inverse of the composite transform described in the input file.
4. Optional cleanup: `make clean`.

## Part 3 – Transformed Object Loader
1. `cd hw0/Part3`
2. Compile with `make` (produces the `load_transformed_objects` executable).
3. Run with a mapping/transform file, e.g. `./load_transformed_objects test_file1.txt`.
   * The input first maps object names to OBJ paths, then lists the sequence of transforms per instance.
4. Optional cleanup: `make clean`.

> **Note:** See the comment in `Part3/load_transformed_objects.cpp` (lines ~129–131) calling out uncertainty about whether the final transformation should multiply by `M` or `M^{-1}`. Revisit that assumption if your results look incorrect.

## Part 4 – PPM Test Pattern
1. `cd hw0/Part4`
2. Compile with `make` (produces the `ppm_test` executable).
3. Generate an image by specifying the desired resolution, e.g. `./ppm_test 512 512 > image.ppm`.
   * Redirect stdout to a `.ppm` file to save the generated image.
4. Optional cleanup: `make clean`.
