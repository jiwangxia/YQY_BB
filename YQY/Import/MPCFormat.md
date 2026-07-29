# MPC input format

```text
*MPC, count
MPC_ID, MasterNode, SlaveNode, SlaveDirection
```

`SlaveDirection` is parsed as a string so that leading zeroes are
retained.

- A single translational direction (`0`, `1`, or `2`) creates one
  constant-distance equation. The selected direction is the dependent
  slave degree of freedom used by nonlinear elimination.
- `05` creates the two coupled equations for the planar nonlinear
  shear-force release in Boungard and Wackerfuss (2024), Eq. (45):
  equal rotation about global Z and zero relative displacement along
  the rotating beam axis. The solver may internally switch the
  translational elimination pivot between slave X and Y near 90-degree
  rotations; the input and stored direction signature remain `05`.
- `012` creates three relative-position equations. The reference offset
  is computed automatically from the initial node coordinates.

Example:

```text
*MPC, 1
1, 1, 2, 0
```
