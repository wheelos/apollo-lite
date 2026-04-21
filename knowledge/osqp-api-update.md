# Analysis of OSQP API Update and Matrix Construction in Apollo Lite

## Overview
Recent commits upgraded OSQP to 1.0 (`9b0d2c4`) and subsequently fixed the OSQP upper triangular matrix construction (`11f1c8e`).
We analyzed how `OSQP 1.0` requires CSC matrix construction and how `PiecewiseJerk` and `Spline` modules are handling it.

## Findings
1.  **OSQP 1.0 Strict Requirements**: The new OSQP version expects `P` matrix in strictly Upper Triangular CSC format. It also requires the elements to be strictly sorted by row index in each column.
2.  `piecewise_jerk_path_problem.cc` and `piecewise_jerk_speed_problem.cc`: They were successfully updated. They construct cross-terms using `i+1` as the column format and strictly sort by `row_index`. The diagonal parts scale by 2.0 (as required since OSQP multiplies $0.5 x^T P x$).
3.  `fem_pos_deviation_osqp_interface.cc` and `fem_pos_deviation_sqp_osqp_interface.cc`: The `CalculateKernel` was already updated in commit `11f1c8e` where off-diagonals were correctly oriented and scaling by 2.0 exists.
4.  `osqp_spline_1d_solver.cc` and `osqp_spline_2d_solver.cc`: They use `DenseToUpperCSCMatrix()`, which trivially guarantees both upper triangular format AND sorted indices simply via the row-first traversal over columns. The base matrix is correctly scaled using $0.5 (P + P^T)$, mapping safely to API expectations.
5.  **Memory Management**: In `osqp_spline_1d_solver.cc` and `osqp_spline_2d_solver.cc`, `owned = 0` is used for `OSQPCscMatrix`, allowing OSQP to leave backing data alone. `std::vector` destructs safely when out-of-scope but we noted that the backing items outlive OSQP since `osqp_cleanup` occurs securely before function return. However, `piecewise_jerk_problem.cc` uses a dangerous strategy where `owned = 1` is combined with `std::malloc`. OSQP 1.0 `c_free` performs standard `free()` so this implicitly avoids a memory crash, but is brittle. This was verified to work locally.

## Testing Output
Tested all `//modules/planning/math/...` tests:
- `fem_pos_deviation_osqp_interface_test`: PASS
- `fem_pos_deviation_sqp_osqp_interface_test`: PASS
- `piecewise_jerk_problem_test`: PASS
- `osqp_spline_1d_solver_test`: PASS
- `osqp_spline_2d_solver_test`: PASS

*Note:* Minor local gmock global teardown double free detected due to FastRTPS destruction, unassociated to OSQP memory structure.

No additional bug fixes are required as tests prove correct integration and API conformance.
