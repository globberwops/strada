# ADR 0005: Cross Section Surface Representation in CPM

## Status
Accepted

## Context
ASAM OpenDRIVE 1.8.0+ introduced the `<crossSectionSurface>` element inside `<lateralProfile>` as a mutually exclusive alternative to traditional `<superelevation>` and `<shape>` profiles. It allows defining the road's lateral elevation profile via a series of strips whose height is determined by polynomials in the lateral coordinate $t$, whose coefficients are themselves piecewise polynomials in $s$.

To support modern maps utilizing OpenDRIVE 1.8/1.9 features, the CPM must support `<crossSectionSurface>` in its core layout and coordinate transformation pipelines without compromising the performance, concurrency, or data-locality constraints established in ADR 0004.

## Decisions

### 1. Flat Struct-of-Arrays (SoA) Storage
We will store all cross-section surface components using flat parallel vectors and CSR-style indexing. This avoids any runtime allocations, pointer-chasing, or virtual dispatch on the hot path.

* **`PolynomialsSoA`**: A single global flat storage of polynomial segments in $s$ (`s_start`, `a`, `b`, `c`, `d`). All width, coefficient, and $t$-offset profiles across all roads are flattened into this structure.
* **`StripsSoA`**: A single global flat storage of strip details across all roads:
  * `strip_id` (`int32_t`): Identifies the strip and side.
  * `is_relative` (`bool`): True if `@mode="relative"`, false if `@mode="independent"`.
  * Index and count ranges (into `PolynomialsSoA`) for `width(s)`, `C0(s)`, `C1(s)`, `C2(s)`, and `C3(s)`.
* **Road-level Indexing**: Per-road arrays (`road_css_first_strip_idx`, `road_css_strip_count`) map a given `RoadId` index to its range of strips in `StripsSoA`. Roads using traditional superelevation/shape will have a `strip_count` of `0`.

### 2. O(1) Sequential Strip Evaluation
The ASAM OpenDRIVE XSD schema enforces a maximum of 4 strips per `<surfaceStrips>` element (`maxOccurs="4"` on `<xs:element name="strip" .../>` inside `t_road_lateralProfile_crossSectionSurface_surfaceStrip`). Because the number of strips per road is extremely small and bounded, the CPM will perform a simple sequential search of strips during evaluation. This eliminates the need for any complex spatial indexing or binary search over strips.

### 3. Signed Lateral Coordinate Evaluation
The independent variable $dt$ used to evaluate a strip's polynomial is signed:
* **Left-side strips ($id > 0$)**: The coordinate $dt$ is positive ($t_{effective} \ge 0$).
* **Right-side strips ($id < 0$)**: The coordinate $dt$ is negative ($t_{effective} < 0$).

### 4. Support for Relative and Independent Modes
Strips with $|id| = 2$ can specify an evaluation mode:
* **`independent`** (default): The height is evaluated purely using the strip's own polynomials:
  $$h(s, t) = h_{strip}(s, dt)$$
* **`relative`**: The height is relative to the outer edge of the inner strip. The CPM will evaluate the inner strip ($|id|=1$) at its full width boundary and add that value to the outer strip's evaluation:
  $$h(s, t) = h_{inner}(s, \pm W_{inner}(s)) + h_{outer}(s, dt)$$

### 5. Roll Orientation is Zero for Cross-Section Surface Roads
Because `<crossSectionSurface>` is mutually exclusive with `<superelevation>` and `<shape>`, a road using a cross-section surface has no banking roll profile. The natural roll (superelevation) angle $\psi(s)$ of the road frame is defined as $0.0$ at all points.

### 6. Unspecified Coefficients Default to $0.0$
Per the XML schema specification, any optional polynomial coefficients ($a, b, c, d$) that are absent in the source `.xodr` file default to `0.0`. The AST parser and CPM build factory will explicitly enforce this fallback.

## Consequences

### Positive
* **Full OpenDRIVE 1.8/1.9 compatibility**: Enables Strada to losslessly load and query modern high-precision map files.
* **Allocation-free and lock-free execution**: The evaluation math touches only contiguous, aligned arrays, maintaining the sub-microsecond latency goal on the 1 kHz hot path.
* **No performance overhead for standard maps**: Roads utilizing standard superelevation bypass the cross-section surface check completely via the $O(1)$ `road_css_strip_count` check.

### Negative
* **Build complexity**: The build factory (`BuildCompiledPhysicsModel`) must validate strip connectivity and resolve defaults/modes during compilation.
