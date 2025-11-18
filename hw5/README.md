# HW5 Implicit Fairing

This program renders a mesh scene and applies implicit fairing to smooth the geometry. Run with:

```
./smooth [scene_description_file.txt] [xres] [yres] [h]
```

Press **`f`** to run the implicit fairing step using the provided time step `h`. Press **`q`** or **Esc** to quit.

## Building the fairing operator

For each vertex with mixed area \(A\), the discrete cotangent Laplacian uses

\[
(\Delta x)_i = \frac{1}{2A} \sum_j (\cot \alpha_{ij} + \cot \beta_{ij})(x_j - x_i),
\]

where \(\alpha_{ij}\) and \(\beta_{ij}\) are the angles opposite the edge \((i, j)\) in the two incident triangles. The implicit system uses \(F = I - h\Delta\), so off-diagonal entries become \(-h\, w/(2A)\) and the diagonal accumulates \(1 + h\,\sum w /(2A)\), matching the construction in `smooth.cpp`.
