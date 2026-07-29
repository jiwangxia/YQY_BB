# Le et al. (2012) Example 4 - Rotating beam

This verification reproduces Fig. 8 and compares the free-end response
envelopes with Fig. 9 of:

T.-N. Le et al., "Dynamics of 3D beam elements in a corotational context",
Finite Elements in Analysis and Design 61 (2012) 97-111.

Parameters:

- straight beam, length 10, 10 equal corotational beam elements;
- left-end translations and rotations about x/y fixed; rotation about z free;
- `A_rho = 1`, `J_rho = diag(20, 10, 10)`;
- `EA = GA = 1e4`, `EI = GJ = 500`;
- `dt = 0.20 s`, total time `30 s`;
- free-end `Fy`: linear ramp 0 to 15 during 0-1 s, then constant;
- free-end `Fz`: triangular pulse 0-15-0 during 0-2 s.

Run:

```text
YQY.exe --verify-le2012-example4
```

The generated time history is written to:

```text
output/verification/le2012_example4/response.csv
```
