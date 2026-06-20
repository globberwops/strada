# ADR 0006: Factoring Out CompiledPhysicsModel Subcomponents

## Status
Accepted

## Context
As the `CompiledPhysicsModel` (CPM) has grown to support cross-section surfaces and multi-strip lateral shapes, it has accumulated a large number of Struct-of-Arrays (SoA) vectors, helper math functions, and compilation routines. This coupling makes it hard to maintain the code, write isolated unit tests, and reason about the separate logical domains (reference line plan geometry, vertical/shape profile, and lane networking).

To improve modularity and align with our development guidelines, we will encapsulate these subcomponents into dedicated classes, mirroring the existing `ReferenceLine` architecture.

## Decisions

### 1. Grouping and Extraction of Subcomponents
We will split CPM's SoA storage and logic into two new independent components:
*   **`ElevationProfile`**: Owns `ElevationSoA`, `ShapesSoA`, and its own `PolynomialsSoA` copy. It compiles and evaluates vertical road features (elevation, pitch, roll, and lateral shapes).
*   **`LaneNetwork`**: Owns `LanesSoA`, `LaneSectionsSoA`, `LaneWidthsSoA`, `LaneHeightsSoA`, `LaneOffsetsSoA`, `RoadCrossSectionSurfaceSoA`, `StripsSoA`, and its own `PolynomialsSoA` copy. It compiles and queries lane topologies, lane widths/heights, and cross-section surface offsets.

### 2. Independent Polynomial Data Sharing
To avoid cross-component coupling and keep `ElevationProfile` and `LaneNetwork` cleanly decoupled, each component will maintain its own independent copy of the compiled `PolynomialsSoA` database.

### 3. Integrated Vertical Profile Query
To preserve sub-microsecond evaluation latency on the 1 kHz hot path, `ElevationProfile::Evaluate` will compute elevation, pitch, roll, and shape height in a single pass. This allows us to share the cost of the binary station searches (e.g., finding the active segment at station $s$) across these queries.
The query returns a unified `VerticalProfile` structure:
```cpp
struct VerticalProfile {
  double elevation{};
  double pitch{};         // natural pitch (std::atan(d_elev))
  double natural_roll{};  // natural roll (from superelevation)
  double roll_total{};    // total roll (natural_roll + std::atan(shape_grad))
  double shape_height{};
};
```

### 4. Compilation & Static Factory Relocation
*   The free function `BuildCompiledPhysicsModel` is converted to a static method `CompiledPhysicsModel::Build`.
*   We delegate compilation responsibilities to static `Build` factories on `ElevationProfile` and `LaneNetwork` respectively.
*   `CompiledPhysicsModel::Build` will orchestrate the compilation by calling:
    1.  `ReferenceLine::Build`
    2.  `ElevationProfile::Build`
    3.  `LaneNetwork::Build`
    4.  Local road string ID and road length caching.
    5.  Spatial bounding volume hierarchy (BVH) construction.

### 5. Thin Dispatches for Transformations
To keep coordinate transformations available to the consumer in one place next to each other, CPM will continue to expose `RoadToInertial`, `InertialToRoad`, `LaneToInertial`, `InertialToLane`, `RoadToLane`, and `LaneToRoad`. However, CPM will serve as a thin facade for lane-related queries, delegating them directly to `LaneNetwork`.

## Consequences

### Positive
*   **Encapsulation & Cohesion**: CPM becomes a high-level manager class, making it easier to read and modify.
*   **Test Isolation**: We can write dedicated unit tests for `ElevationProfile` and `LaneNetwork` without instantiating the full coordinate transform machinery.
*   **Performance Maintenance**: The single-pass `VerticalProfile` design avoids redundant binary searches on the hot path.

### Negative
*   **Polynomial Duplication**: Polynomial coefficient vectors are stored twice (once in each subcomponent), but this has negligible memory impact.
