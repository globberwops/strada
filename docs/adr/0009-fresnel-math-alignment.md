# ADR 0009: Fresnel Integrals Clamping and Minimax Approximation Alignment

## Status
Accepted

## Context
The Compiled Physics Model (CPM) requires extremely fast, branch-free, and vectorized evaluations of Fresnel sine $S(z)$ and cosine $C(z)$ integrals to support vehicle dynamics queries on clothoid transitions. Strada's current Fresnel solver uses a dynamic `do-while` loop for the small inputs domain ($|z| < 1.0$) and evaluates asymptotic expansions to infinity without checking upper boundaries. While mathematically precise, this loop-based approach prevents compiler SIMD auto-vectorization, and out-of-bounds queries can cause slow convergence or numerical overflow.

## Decision
We decided to align Strada's Fresnel coordinate solver directly with the ASAM OpenDRIVE Cephes-based mathematical reference implementation:
1. **Loop-Free Small Domain**: Replace the dynamic Taylor series expansion for the small input domain ($|z| < 1.6$) with Cephes-style minimax rational Chebyshev polynomials (degree 5 over 6). This eliminates loop branches, guaranteeing $O(1)$ constant-time execution and enabling efficient SIMD auto-vectorization.
2. **Asymptotic Upper Clamping**: Introduce a hard clamping threshold matching Cephes at $|z| > 36974.0$. Inputs exceeding this limit immediately return $C(z) = 0.5$ and $S(z) = 0.5$, protecting the asymptotic series solver from numerical instability and execution hangs.
3. **Domain Compatibility**: Retain the completed-square integration technique (`EvalXYaLarge`) to directly evaluate generalized clothoid offset coordinates for arbitrary initial curvatures ($\kappa_0 \neq 0$), ensuring Strada remains more robust than the reference library's zero-curvature spiral calculation constraint.

## Consequences
- **Performance & SIMD**: Eliminating loop branches in the hot-path math primitives allows full SIMD batch query pipelining on coordinates.
- **Safety and Stability**: Clamping very large inputs prevents execution hangs and double-precision float overflow.
- **Spec Consistency**: Resolving integrals using identical coefficients ensures exact coordinate matching with reference ASAM OpenDRIVE tooling.
