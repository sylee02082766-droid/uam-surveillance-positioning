# Validation record

Public-source preparation: 2026-10-07.

`run_demo` was executed with the installed MATLAB R2024b. The three runs completed the finite-error assertions and printed this result on the included **synthetic** obstacles, with a nominal reference separated from the perturbed actual trajectory:

| Method | Demo RMSE [m] | Demo P95 [m] |
|---|---:|---:|
| Baseline | 233.08 | 653.02 |
| A1 | 6.3400 | 15.908 |
| A2 | 5.6739 | 15.254 |

These numbers only demonstrate that the public example executes and that the filter variants differ in this deliberately fabricated scenario. They are **not** the original ICROS/Olympiad measurements or benchmark and do not establish operational accuracy.

The copied algorithm files were checked against source SHA-256 hashes. Private/industry source, original GIS inputs and precomputed paper CSV/PNG results are excluded. The full 50-trial Monte Carlo script and optional plotting/image-composition helpers were not run during this numerical smoke check.
