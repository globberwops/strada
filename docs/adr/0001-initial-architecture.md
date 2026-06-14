# ADR 0001: Initial Architecture for Strada

## Status
Accepted

## Context
Strada is a modern C++ library for ASAM OpenDRIVE 1.9. It must read `.xodr` files, load them losslessly into memory, and support diverse consumers with conflicting requirements:
1. **Vehicle Dynamics (Physics)**: Extremely high frequency (1000Hz+), multi-wheel contact point queries demanding sub-microsecond latency, thread safety, and cache friendliness.
2. **Driver Models & Scenario Engines**: Need logical network connectivity and topology (predecessor/successor links, junctions) for routing.
3. **Visualization Systems**: Require tessellated polylines and 3D meshes of roads/lanes.

To support these use cases efficiently, the codebase needs to be architected with clear boundaries, separating logical schema compliance from performance-oriented math evaluation.

## Decisions

### 1. Separation of AST, Physics Model, and Routing Graph
We will divide the library into three distinct runtime representations:
* **Abstract Syntax Tree (AST)**: A strongly-typed, object-oriented representation of the OpenDRIVE 1.9 schema. To support lossless representation, unknown or custom XML tags and attributes are stored in a dynamic extension map associated with each element.
* **Compiled Physics Model (CPM)**: A decoupled, optimized mathematical representation. Geometry profiles, lane widths, and elevation curves are compiled into flat, aligned memory buffers (Struct-of-Arrays). Real-time spatial indexing is built using a flat, contiguous Bounding Volume Hierarchy (BVH) instead of a pointer-chasing tree.
* **Logical Routing Graph**: A directed graph representing lane-to-lane topology and junction pathways for routing.

### 2. C++20 Standard
We will adopt the C++20 standard. This allows the use of modern performance and design abstractions, such as:
* `std::span` for zero-copy views into contiguous memory buffers (hot-path query interfaces).
* C++20 Concepts for compile-time interface verification.
* C++20 Ranges for clean array transformations.

### 3. Build System & Dependencies
* **CMake** will be the build system.
* **CMake FetchContent** will manage external dependencies at compile time, requiring zero local system pre-installation.
* Dependencies will be restricted to:
  * `pugixml` for fast XML parsing.
  * `Eigen` for linear algebra, vector math, and coordinate transformations.

### 4. Geometry Evaluation & Discretization
* Strada will store the exact mathematical curves (lines, clothoids/spirals, polynomials, parametric cubics) analytically.
* We will provide exact coordinates and derivative evaluation functions (using high-performance approximations for clothoid Fresnel integrals).
* A dynamic tessellator utility will be provided to discretize curves into polylines or mesh buffers using a user-configurable chord error tolerance.

### 5. Pure C++20 API
* Strada will focus 100% on a clean C++20 API.
* We will not maintain Python (pybind11) or other language bindings in the core repository.

### 6. Geodetic Projection
* The `<geoReference>` tag string containing the coordinate projection string (PROJ/WKT) will be parsed and exposed as a raw string to the API.
* Strada will not perform geographic projection math internally, avoiding the inclusion of heavy projection libraries (like `libproj`).

## Consequences

### Positive
* **Physics Performance**: The Compiled Physics Model (CPM) avoids cache misses and pointer-chasing during high-frequency simulation loops.
* **SIMD & Threading Friendly**: Contiguous memory layout (SoA) enables compiler auto-vectorization and vectorized batch queries. The read-only nature of CPM allows lock-free parallel queries across threads.
* **Lossless representation**: The AST retains 100% schema fidelity and custom attributes, allowing complete round-tripping.
* **Low Dependency Footprint**: Easy integration into other CMake projects with minimal compilation dependencies.

### Negative
* **Memory Footprint**: Keeping both the raw AST and the compiled CPM in memory increases memory usage, though this is negligible on modern simulation machines (OpenDRIVE files are typically under 100MB).
* **Double Processing**: Loading a map requires a compile/flatten phase after XML parsing.
