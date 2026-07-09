# Strada

Strada is a C++20 library for the ASAM OpenDRIVE 1.9 format. It loads `.xodr` files into an in-memory abstract syntax tree (AST) and compiles them into flat physics models and topological routing graphs.

## Features

- **AST Parser**: Lossless loading of OpenDRIVE XML schemas into typed C++ structures.
- **Compiled Physics Model (CPM)**: Contiguous, cache-friendly Struct-of-Arrays (SoA) layout for 1 kHz coordinates and pose query loops.
- **Routing Graph**: Topological lane network representation for navigation and pathfinding.
- **Tessellator**: Geometry generation of road surfaces, boundaries, and markings.

## Getting Started

### Prerequisites

You need a compiler with C++20 support, CMake 3.22+, and the dependencies configured in the development container.

### Build and Run Tests

1. Configure the build directory:
   ```bash
   cmake --preset default-debug
   ```
2. Build the project:
   ```bash
   cmake --build --preset default-debug -j$(nproc)
   ```
3. Run the test suite:
   ```bash
   ctest --test-dir build/default --output-on-failure
   ```

## Documentation

- [CONTRIBUTING.md](CONTRIBUTING.md): Commit conventions, C++ style rules, and pre-commit hook setups.
- [CONTEXT.md](CONTEXT.md): System architecture, glossary, and domain representations.

## License

This project is licensed under the Boost Software License 1.0.
