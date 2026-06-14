# ADR 0002: ASAM OpenDRIVE 1.9.0 Specification Alignment & Math Resolutions

## Status
Accepted

## Context
A detailed review of the ASAM OpenDRIVE v1.9.0 specification reveals mathematical and schema details that contradict simple object-tree query implementations. To guarantee sub-microsecond query speeds in the Compiled Physics Model (CPM) and 100% serialization fidelity in the AST, we must clarify our math execution and extensibility strategies.

## Decisions

### 1. Pre-conversion of Legacy Cubic Polynomials (`<poly3>`)
The deprecated `<poly3>` geometry defines lateral offset as a function of the local tangent coordinate $u$, which does not scale linearly with arc length $s$. Calculating coordinates requires solving:
$$s(u) = \int_0^u \sqrt{1 + (v'(u'))^2} du'$$
for $u$ given $s$ using numerical root-finding on every query step.
* **Decision**: The CPM compiler will pre-convert all `<poly3>` curves into `<paramPoly3>` curves (parameterized by arc length) or high-density piecewise arc-line segments at load time. The CPM runtime will not execute numerical root-finding or arc-length integration on the fly.

### 2. Rational Fresnel Approximations for Clothoids (`<spiral>`)
Clothoid evaluation requires Fresnel integrals:
$$S(x) = \int_0^x \sin(t^2) dt, \quad C(x) = \int_0^x \cos(t^2) dt$$
* **Decision**: Rather than using numerical integration methods, the CPM will evaluate Fresnel integrals using fast rational approximations (e.g., Heald's or Cody's methods). This provides a constant-time, branch-free execution path that is highly suited for SIMD auto-vectorization.

### 3. Separation of Superelevation and Bivariate Road Shape Profiles
Road elevations can be specified via simple superelevation or complex `<shape>` profiles. The `<shape>` profile defines heights as piecewise polynomials in the lateral coordinate $t$, which are then linearly interpolated in the longitudinal direction $s$.
* **Decision**: 
  1. The CPM will compile `<shape>` stations into a contiguous array of lateral polynomial groups.
  2. For roads using `<shape>` profiles, CPM will locate the surrounding stations via flat interval lookups and interpolate their results.
  3. Roads utilizing only standard `<superelevation>` (which covers the vast majority of maps) will bypass this bivariate shape check entirely.

### 4. Lossless `<userData>` and Custom Attribute Storage
The OpenDRIVE `<userData>` element permits arbitrary nested XML namespaces, and datasets often contain custom XML attributes.
* **Decision**: The AST's extension container will store:
  1. Custom XML attributes as flat `std::unordered_map<std::string, std::string>`.
  2. Nested XML nodes under `<userData>` as raw XML string blocks or lightweight `xml_node` DOM fragments, guaranteeing complete serialization fidelity when export is requested.

## Consequences
* **CPM Math Execution**: Constant-time $O(1)$ reference line evaluation is guaranteed by replacing numerical root-finding and integration with pre-conversions and rational approximations.
* **Serialization Integrity**: Storing nested XML blocks in the AST ensures that non-schema-compliant customer data is preserved 100% when saving back to `.xodr` formats.
