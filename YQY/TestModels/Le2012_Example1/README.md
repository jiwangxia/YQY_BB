# Le et al. (2012) Example 1 - Right-angle cantilever

This verification reproduces Fig. 2 and compares the out-of-plane
tip and elbow responses with Fig. 3 of:

T.-N. Le et al., "Dynamics of 3D beam elements in a corotational context",
Finite Elements in Analysis and Design 61 (2012) 97-111.

Parameters:

- right-angle cantilever with two members of length 10;
- 10 equal corotational beam elements per member, 20 elements total;
- fully clamped base;
- `A_rho = 1`, `J_rho = diag(20, 10, 10)`;
- `EA = GA = 1e6`, `EI = GJ = 1e3`;
- `dt = 0.25 s`, total time `30 s`;
- elbow `Fz`: triangular pulse 0-50-0 during 0-2 s;
- standard Newmark average-acceleration method, `beta = 0.25`,
  `gamma = 0.5`.

Run:

```text
YQY.exe --verify-le2012-example1
```

The time history and comparison summary are written to:

```text
output/verification/le2012_example1/response.csv
output/verification/le2012_example1/summary.txt
```
