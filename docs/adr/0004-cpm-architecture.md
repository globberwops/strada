# ADR 0004: Compiled Physics Model (CPM) Architecture

## Status
Accepted

## Context

ADR 0001 sketched the *existence* of the Compiled Physics Model as a separate, optimized runtime representation, and ADR 0002 nailed down the math-level decisions (poly3 pre-conversion, rational Fresnel approximations, bivariate shape caching, lossless extensions). What ADR 0001 and 0002 deliberately *did not* settle is the *data layout* and *public API shape* of the CPM — the questions that determine whether the 4–8 wheels × 1 kHz hot path actually delivers sub-microsecond per-query latency on a stable vehicle.

This ADR records the design decisions taken in `docs/agents/` design sessions to close that gap. It complements ADR 0001 (the *what*) and ADR 0002 (the *math*) with the *how*: the storage layout, the pose types, the query API, the spatial index, and the threading model.

The driving constraint, restated: the primary consumer of the CPM is a vehicle dynamics loop at 1 kHz, with 4–8 wheel contact-point queries per timestep per vehicle. The hot path must be allocation-free, lock-free, branch-friendly, and cache-local. Everything else (batch queries, mixed precision, mixed representation) is deliberately deferred.

## Decisions

### 1. Lifetime and Ownership — the CPM is Fully Self-Sufficient
The CPM is built from a `const ast::OpenDrive&` and returns a self-sufficient `CompiledMap` value that owns all of its data. The CPM holds **no pointers or references** into the AST. The AST can be dropped immediately after `BuildCompiledPhysicsModel` returns, or kept alongside for AST-only operations (lossless round-trip export, debugging, the Routing Graph and Tessellator layers). The two representations are peers, not coupled.

### 2. Build Factory Convention
Each layer (AST, CPM, Routing Graph, Tessellator) is constructed by a uniform `Build*` free function:
```cpp
auto BuildAbstractSyntaxTree (const std::filesystem::path&) -> ast::OpenDrive;
auto BuildCompiledPhysicsModel(const ast::OpenDrive&)      -> cpm::CompiledMap;
auto BuildLogicalRoutingGraph(const ast::OpenDrive&)       -> routing::Graph;
```
Each factory takes its upstream input by `const` reference and returns a fully-owned value. Factories are individually composable. A planned `Map` facade will chain them from a single `.xodr` file but does not change the per-layer contract.

### 3. The Full-SoA Principle (non-negotiable)
The CPM is **fully Struct-of-Arrays throughout** — no ragged nested structures, no `std::vector<std::vector<...>>` of any kind, no pointer-chasing through the hot path. Every datum the hot path touches lives in a flat array; navigation uses pre-computed offset tables, not runtime lookups. If a concept cannot be efficiently represented in flat SoA, it does **not** belong in the CPM — it stays in the AST or a different layer.

This is a non-negotiable design principle: the CPM exists *because* of this flattening, and a partial commitment to SoA would not justify a separate compiled layer over the AST.

### 4. SoA Granularity — per Road
SoA arrays are organized *per road*: each road has its own geometry, lane-section, elevation, and lateral-profile arrays. The BVH is the *one* global structure spanning all roads. A road's data is contiguous in memory, which is what the 1 kHz single-road hot path needs. Cross-road batch operations (rare in this consumer) are *not* a v1 optimization target.

### 5. Top-Level Type — a Single `CompiledMap`
`CompiledMap` is a single value type that owns everything (geometry, lanes, profiles, BVH, road-id translation tables). Roads are not separately addressable value types. A user calls `cpm.road_count()`, `cpm.road_id_from_string("…")`, or one of the transformation methods. Per-road data lives inside `CompiledMap`'s SoA arrays and is addressed by `RoadId` / `LaneId`.

### 6. Identifier Types — Strongly-Typed Integer Handles
`RoadId` and `LaneId` are both `enum class : std::uint32_t` — strongly typed, dense over `[0, N)`, no implicit conversion to/from integers. A `RoadId` is the *compiled* handle (an internal index); the AST's string `road.id` is the *original* id, distinct from the compiled handle. A `LaneId` is a *global* handle (encodes the road internally), so `LaneToInertial(LaneId, …)` takes a single identifier, not a `(RoadId, LaneId)` pair.

### 7. Three Flat Pose Types, All 6-DoF
The CPM exposes three first-class pose types — `InertialPose`, `RoadPose`, `LanePose` — each a single struct with all 6 DoF as direct members, no `*Position` or `*Orientation` sub-types on the public surface:
```cpp
struct InertialPose { double x, y, z;           double heading, pitch, roll; };
struct RoadPose    { double s, t, h;           double heading, pitch, roll; RoadId road; };
struct LanePose    { double s, t, h;           double heading, pitch, roll; RoadId road; LaneId lane; };
```
Field order is **position first, then orientation, then IDs at the end** — keeps the 6 DoF contiguous in all three types and makes SIMD-batch strides predictable.

### 8. The Orientation Convention — Pose Holder's Orientation in Frame
The orientation fields in every pose type describe the **pose holder's** orientation in that frame's coordinates, *not* the frame's orientation in the world. A pose aligned with its frame has `heading = pitch = roll = 0`. This is the only interpretation under which all six pairwise transformations are meaningful — `RoadToInertial(rp).orientation` is the world-frame orientation of composing the road's local frame with the pose holder's offsets.

### 9. Six Pairwise Transformations
The CPM exposes all six pairwise transformations between the three pose types, named `FromTo`:
| Pair | Forward (always) | Inverse / cross (may fail) |
|---|---|---|
| Road ↔ Inertial | `RoadToInertial` | `InertialToRoad → std::optional<RoadPose>` |
| Lane ↔ Inertial | `LaneToInertial` | `InertialToLane → std::optional<LanePose>` |
| Lane ↔ Road | `LaneToRoad` | `RoadToLane → std::optional<LanePose>` |

The three *forward, deterministic* transformations are pure math on the input pose. The three *inverse / cross* transformations involve a search (`InertialTo*` uses the BVH, `RoadToLane` walks the road's lane tree) and return `std::nullopt` on failure.

### 10. Coordinate System — Z-up ENU, Intrinsic Z-Y-X Euler
The inertial frame follows the **geographic ENU convention** per ASAM OpenDRIVE 1.9.0 § 8.2: `x` = East, `y` = North, `z` = Up (vertical height above ground). The orientation is **intrinsic body-axis Z-Y-X (Tait-Bryan) Euler** (heading, then pitch, then roll), right-hand rule around each body axis, in **radians**. The resulting rotation matrix is `R = R_z(heading) · R_y(pitch) · R_x(roll)`, carrying body-frame vectors to the world frame.

### 11. Pitch in the Road Frame — Offset from Natural Pitch
ASAM OpenDRIVE 1.9.0 § 8.3 states *"For the s/t/h coordinate system no pitch is possible."* The road's natural pitch is derived from the elevation profile gradient: `pitch_road(s) = atan(d_elev(s)/ds)`. The `pitch` field in `RoadPose` is the **pose holder's offset** from the road's natural pitch. `RoadToInertial` composes the road's natural orientation (heading from tangent, pitch from elevation, roll from superelevation) with the input's offsets. A `RoadPose = {0, 0, 0, 0, 0, 0, road}` therefore yields the road's *natural* pose, the right default for inspection.

### 12. Reference Line Representation — Per-Shape Parameter Arrays
Per road, the reference line is stored as a flat array of segments with **per-shape parameter arrays** (a per-type SoA):
- Unified metadata array: `s_offset`, `length`, `x`, `y`, `hdg`, `type`, `type_index` — one entry per segment.
- Per-type parameter arrays: `arc_curvature`, `spiral_curv_start`/`curv_end`, `pp3_a_u..pp3_d_v`/`pp3_p_range` — sized to the count of segments of that type.
- No "all-padded" alternative: a line is a line, an arc has just curvature, and we don't waste a `double[9]` per segment.
- **Poly3 is pre-converted to `paramPoly3` (arc-length parameterization)** at compile time. No on-the-fly arc-length integration in the runtime.
- **Spirals are kept as `Spiral`** with rational Fresnel approximation per ADR 0002. No chord-error arc-line conversion; one segment per spiral.

### 13. Lane Section Representation — Per-Road SoA with CSR Offsets
Per road, the lane structure is a flat SoA with CSR-style offset tables:
- Sections: flat arrays of `section_s`, `section_first_lane_idx`, `section_lane_count`.
- Lanes: flat array of `lane_id` (signed int), `lane_first_width_idx`, `lane_width_count`.
- Width polynomials: flat arrays of `width_s_offset`, `width_a..width_d`.
- Heights: same pattern (`height_s_offset`, `height_inner`, `height_outer`) — kept for completeness.
- **AST-only fields are dropped**: `Lane.type`, `Lane.level`, `Lane.predecessor`, `Lane.successor` are not carried into the CPM. The signed `Lane.id` is kept for inspection.
- Lane offset polynomial: same SoA pattern as the elevation profile.

### 14. Spatial Index — Flat BVH, 2D AABBs, Binned SAH
The BVH is a **single global flat array** of nodes, built at compile time over the roads' 2D plan-view AABBs:
- Each node: 4 doubles for the 2D AABB + two `uint32_t` for child indices. The `is_leaf` flag is bit-packed into the high bit of `right` (40 bytes/node, 1.6 nodes per 64-byte cache line).
- 2D AABBs (plan-view x/y only) are preferred over 3D: smaller, the BVH's job is to *narrow down* the road, and the final `s/t/h` check inside the road validates `z`.
- Construction uses **binned SAH** for `O(N log N)` build with high-quality traversal.
- The BVH is the only global structure in the CPM; everything else is per-road.

### 15. Query Context — Per-Thread, Stateful Hot Path
Hot-path queries take a `QueryContext&` parameter — a small (~40 byte) consumer-owned value type that carries temporal-coherence state (last `RoadId`, last `s`-range with slack, last segment, last BVH node). The dominant usage pattern is `thread_local QueryContext ctx;` at the top of the simulation loop.

For a stable vehicle, the fast path is: two `RoadId` comparisons + one `s`-range check + one Horner polynomial evaluation ≈ 15–25 ns per query. The slow path (road change, first query) does a full BVH traversal at ~1–5 µs and is rare.

Hot-path queries are **`noexcept`**, designed to be aggressively inlined, and marked `[[gnu::hot]]` to bias the optimizer. Sharing a `QueryContext` across threads is undefined behavior (consumer's responsibility).

### 16. v1 Scope — Single-Query API Only
v1 is single-query only. There is **no batch API** (`std::span` overloads are not exposed). The driving use case is 4–8 wheels per vehicle at 1 kHz; batch SIMD across wheels is not the bottleneck, and the QueryContext fast path captures the relevant temporal coherence. Batch overloads can be added later without breaking the single-query API.

### 17. Numerical Precision — `double` Throughout
All SoA arrays store `double`. The 2× SIMD throughput from `float` is not worth the precision loss for any map larger than ~10 km (where `float`'s ~10 m precision is coarser than a single road's width). The AST already uses `double`; matching it is the trivially-correct choice. Mixed precision can be added later as a compile-time option once benchmarks show it matters.

SoA arrays are aligned to 64-byte cache lines via an `AlignedAllocator<T, 64>` to enable `[[assume_aligned]]` and reliable auto-vectorization on AVX2/AVX-512.

### 18. No `CompileOptions` for v1
The factory signature is the simplest possible:
```cpp
auto BuildCompiledPhysicsModel(const ast::OpenDrive& map) -> CompiledMap;
```
There is no `Validation` knob, no `num_threads` knob, no `max_memory_bytes` knob, no precision knob, no BVH-quality knob. Validation is the *parser/AST layer's* concern, not the CPM's; the CPM trusts its input. Knobs we considered (precision, BVH quality, keep-extensions, etc.) are all deferred until a real consumer needs them.

### 19. Error Handling — Build Throws, Hot Path is `noexcept`
The split is sharp:
- **`BuildCompiledPhysicsModel` may throw** on resource exhaustion or unrecoverable internal errors. The CPM does not perform AST validation; the parser/AST layer is responsible for input well-formedness.
- **Hot-path queries are `noexcept`.** Failure modes for the inverse / cross queries are communicated via `std::optional<...>` (`std::nullopt` on failure), never via exceptions. A `noexcept` query in a `noexcept` hot loop is critical — exceptions would `std::terminate`.

### 20. Threading Model — Immutable after Build, Lock-Free Reads
Once `BuildCompiledPhysicsModel` returns, every byte of the `CompiledMap` is read-only. There are no locks, no atomics, no mutable state inside the CPM. The per-thread `QueryContext` is the *only* mutable state and is consumer-owned. Hot-path reads (`RoadToInertial`, etc.) are lock-free and wait-free: a call from any thread touches only the immutable `CompiledMap` and that thread's `QueryContext`. Two threads can call the same `CompiledMap`'s hot-path methods concurrently with no synchronization.

### 21. Eigen — Kept in `external/`, Not Used in the Hot Path
Eigen remains a `FetchContent` dependency (per ADR 0001) but is **not used in the CPM's hot path for v1**. The arithmetic is short, branchy, and dominated by memory access — the opposite of where Eigen shines. The CPM's 3D rotation math, polynomial evaluation, and AABB checks are implemented in a small private `rotation.hpp` using raw `double` arrays and `std::array<std::array<double, 3>, 3>` for 3×3 matrices. Eigen can be pulled into a specific translation unit later (e.g. for batch evaluation or tooling) without churning the public API.

## Consequences

### Positive
* **Sub-microsecond per-query latency on the hot path.** The combination of full SoA, 64-byte alignment, aggressive inlining, `noexcept` + `[[gnu::hot]]` annotations, the QueryContext fast path, and the avoidance of any heap allocation in `RoadToInertial` / `LaneToInertial` puts the per-query cost at 15–25 ns for a stable vehicle, with a 1–5 µs tail on road changes.
* **Lock-free concurrency out of the box.** Immutable CPM data + per-thread QueryContext = trivial thread safety, no synchronization primitives anywhere in the hot path.
* **SIMD-friendly throughout.** Per-shape parameter arrays, 64-byte alignment, no jagged data, no virtual dispatch, and `[[likely]]` / `[[unlikely]]` hints on the fast-path branch let the compiler auto-vectorize aggressively.
* **Self-sufficient artifacts.** Each layer (`CompiledMap`, `routing::Graph`, `tess::Tessellator`) is fully owned; the AST can be dropped after building, or kept for AST-only operations. A binary-format serialized map is a real future option.
* **Clean separation of concerns.** Validation is the parser's job; math is the CPM's job; topology is the Routing Graph's job. The 21 decisions above are all *narrow* in scope.
* **Reversible precision/strategy decisions.** The deferred knobs (mixed precision, BVH quality, poly3 strategy) can be added later as `CompileOptions` without changing the pose types, the QueryContext, or the public API.

### Negative
* **Memory footprint.** A road's data is duplicated in the SoA's natural-size form (e.g. a line segment is just the metadata, ~32 bytes; in the AST it's a `GeometryRecord` + `std::variant` tag, which is larger). For typical maps this is still well under 100 MB, but the redundancy is real.
* **Index-table complexity.** Lane section SoA carries CSR offset tables (`section_first_lane_idx`, `section_lane_count`, `lane_first_width_idx`, `lane_width_count`). Building and maintaining these is more code than a ragged representation; the payoff is cache locality on the hot path.
* **Stateful hot-path API.** The `QueryContext&` parameter on every hot-path method is a real ergonomic cost — every simulation loop has to declare and pass one, and a missed declaration falls back to the slow path silently. The trade-off is the 10–100× speedup on the fast path; we accept it.
* **Pose semantics are subtle.** "Orientation = pose holder's orientation in the frame" is the only interpretation that makes all six transformations meaningful, but it's also the interpretation that's most easily misread. The glossary entry on `Pose holder's orientation in a frame` exists specifically to keep this straight.
* **No batch API in v1.** Tooling that wants to evaluate 10,000 points at once has to call `RoadToInertial` 10,000 times. For v1 we accept this; a batch API can be added later without breaking the single-query API.

### Constraints Recorded but Not Yet Implemented
* **Rational Fresnel approximation for spirals** — required for `Spiral` evaluation; ADR 0002 already pins this but the actual coefficients and accuracy bounds will be settled at implementation time.
* **Binned SAH parameters** — number of bins, traversal cost model, etc., will be settled at BVH implementation time.
* **Poly3 → `paramPoly3` conversion** — the algorithm for reparameterising the polynomial in arc length is well-known; implementation details (root-finding tolerance, fallback) will be settled at compile-time.
* **Poly3 pre-conversion edge cases** — poly3 segments with `b == c == d == 0` (degenerate), `a == 0` (zero offset), NaN coefficients; the parser should already reject the latter, but the build should be defensive.
