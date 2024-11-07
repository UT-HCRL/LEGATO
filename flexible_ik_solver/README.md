# Flexible IK Solver

**Flexible IK Solver** is a cross-language library for a flexible Inverse Kinematics (IK) solver. This solver is a variant of the extended-SNS-IK velocity IK solver [1]. It enables you to solve the inverse kinematics problem for robots with arbitrary kinematic structures and constraints, including joint limits and collision constraints.

The library is designed to be easily integrated into your projects, and it supports both Python and Julia. The solver implemented in Julia can be called from Python using the `JuliaCall` package, which allows you to use the solver in a Python environment.


## Supported Languages
- Python
- Julia

## Requirements
- Python 3.6+

## Installation
The module is installed during `bdai install bdai`.
It can be used via, e.g.,
```python
from flexible_ik_solver import flex_ik_py
solver = flex_ik_py.InverseSolver(...)
```
or
```python
from flexible_ik_solver import InverseSolver
```

To install the Julia package to use it in Python, use the following script:

```shell
python flex_ik_julia/setup_flex_ik_julia.py
```

## Testing
To ensure the library works as expected, run the tests with pytest:

```shell
python -m pytest
```

For more testing options:
- Use `-v` for verbose output.
- Use `-s` to print out the print statements in the code.

## Usage
Explore the provided examples under the `examples` folder to understand how to integrate this library into your projects.

The example for using Julia implementation (`FlexIK.jl`) in Python scripts is provided in the `example_flex_ik_julia.py` script.

## Author
Andy Park <andypark@theaiinstitute.com>

## References
[1] Fiore, Mario Daniele, Gaetano Meli, Anton Ziese, Bruno Siciliano, and Ciro Natale. "A General Framework for Hierarchical Redundancy Resolution Under Arbitrary Constraints." IEEE Transactions on Robotics (2023).

---

# Changelog
We keep track of notable changes to this project here, following Semantic Versioning. The tags used are as follows:

- `Added` for new features.
- `Changed` for modifications to existing functionality.
- `Deprecated` for features soon to be removed.
- `Removed` for features that are no longer available.
- `Fixed` for bug fixes.

## [0.2.0] - 2024-02-12
### Changed
- Fixed a bug in the box constraint calculation for the velocity IK solver.
- Added unit tests for the box constraint calculation.
- Renamed the methods to use snake case, and updated the examples to reflect the changes.
- Updated docstrings in the code.
- Added Julia implementation of the solvers.

## [0.1.5] - 2023-09-27
### Changed
- Transitioned from `setup.cfg` to `pyproject.toml`.
- Reorganized folder structure (removed `flex_ik_py` folder and moved all files to the root folder).


## [0.1.4] - 2023-09-08

### Added
- Introduced tests with different matrix inverse solvers.
- Added support for joint saturation constraints.

### Changed
- Transitioned from `setup.py` to `setup.cfg`.
- Restructured the code to utilize classes for all functions.
- Updated examples to reflect the API changes.

## [0.1.3] - 2023-09-01

### Added
- Implemented the use of `spdlog` for logging.

## [0.1.2] - 2023-04-07

### Added
- Incorporated PyTest for unit tests in the velocity IK solver.

### Changed
- Replaced traditional matrix multiplication with the `@` operator.

### Fixed
- Resolved a minor initialization bug.
