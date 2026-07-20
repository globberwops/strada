---
name: diagnostic-sweep
description: Clean clangd findings across a file or directory with per-file commits.
disable-model-invocation: true
---

# Sweep

**Relentless sweep.** Make every file _clean_ — zero clangd findings, build green, tests green, committed.

**Channel: clangd via `xd://lsp` exclusively.** If `xd://lsp` is _wedged_ — two consecutive errors on the same file after a reload — the sweep stops.

- Schema, response shape, ordering rules, failure modes → [lsp-tool.md](lsp-tool.md)
- Setup troubleshooting → [clangd-setup.md](clangd-setup.md)

## 1. Invocation & Target Handling

Accepts a path argument (`<path>`):

- **File mode:** Process `<path>` directly through [File Processing Loop](#2-file-processing-loop) without creating `diagnostic-sweep.md`.
- **Directory mode (or unspecified, defaulting to `.`):**
  - Glob source and header files recursively inside `<path>`: `*.c *.cc *.cpp *.cxx *.h *.hpp *.hxx *.inl`. Exclude build dirs, `tools/`, third-party, generated.
  - Write `diagnostic-sweep.md` at repo root:

    ```markdown
    | # | File | Status | Note |
    |---|------|--------|------|
    | 1 | src/foo.cpp | ⬜ |  |
    ```

    Status markers: ⬜ pending, ✅ clean, ⚠️ HITL / wedged.
  - Process each row sequentially through [File Processing Loop](#2-file-processing-loop).

## 2. File Processing Loop

Finish each file entirely — diagnose, fix, re-verify, build, test, commit, and update status — before proceeding to the next.

1. `{"action":"diagnostics","file":"<file>"}` via `xd://lsp`. Read each finding.
2. For each diagnostic:
   - Read the cited code.
   - Fix it to achieve a _clean_ file — style nits included.
   - Prefer clang Fix-its (`code_actions`) and `rename` over hand-editing. Query `code_actions` at the finding's line before hand-fixing ([lsp-tool.md](lsp-tool.md)).
   - Route to HITL when the fix requires design decisions or breaks a public API. When unambiguous, fix it directly.
   - **Magic-number rule:** for `cppcoreguidelines-avoid-magic-numbers` and `readability-magic-numbers`, see [Magic-number handling](#magic-number-handling) below.
   - **Cognitive-complexity rule:** for `readability-function-cognitive-complexity`, see [Cognitive-complexity handling](#cognitive-complexity-handling) below.
3. Re-run `{"action":"diagnostics","file":"<file>"}`. Repeat until zero findings.
4. `cmake --build --preset dev-debug && ctest --preset dev-debug` (configure first if no build dir: `cmake --preset dev-debug`). All must pass.
5. `git add` the touched files and commit: `style(diagnostic-sweep): clean <file>`.
6. **If directory mode:** Edit `diagnostic-sweep.md`: row N → ✅. If HITL, → ⚠️ with a one-line Note. If wedged, → ⚠️ and the sweep stops.

### NOLINT suppression (HITL)

When routing a finding to suppression, delegate application to the human:

1. In directory mode, mark row ⚠️ in `diagnostic-sweep.md` with a one-line note (formula + reference for magic numbers; algorithm + justification for cognitive complexity). In file mode, report ⚠️ note in output.
2. Leave warning unedited in file.
3. Skip build, test, and commit — human applies NOLINT, comment, and commit.

### Magic-number handling

When clangd flags `cppcoreguidelines-avoid-magic-numbers` or `readability-magic-numbers`, choose one of two paths:

**Fix (extract named constant).** The default. Apply when constant carries domain meaning independent of formula (tolerances, thresholds, limits, physical constants). Example: `1e-9` → `kMinCurvature`.

**Suppress via NOLINT (HITL).** Apply when constant is *structural* (value fixed by position in formula or series expansion). Per [NOLINT suppression (HITL)](#nolint-suppression-hitl), human applies marker and comment.

The comment must precede the NOLINTNEXTLINE on the same line or in the same block, using `//` (not `/* */`). It must contain:

- The general form of the formula, or its name.
- A reference (textbook, paper, or standard) where the formula lives.

Example:

```cpp
// Taylor series for sin(x): x - x^3/3! + x^5/5! - x^7/7! + ...
// Reference: Abramowitz & Stegun §4.3
// NOLINTNEXTLINE
const double kSeriesCoef = 5040.0;
```

### Cognitive-complexity handling

When clangd flags `readability-function-cognitive-complexity`, choose one of two paths:

**Refactor (extract helper functions).** The default. Apply when complexity is *incidental* (tangled logic, mixed concerns, sequential blocks). Extract until helpers have single responsibilities and findings clear.

**Suppress via NOLINT (HITL).** Apply when complexity is *algorithmic-essential* (branching structure IS the algorithm: state machine, recursive descent, multi-field parser). Per [NOLINT suppression (HITL)](#nolint-suppression-hitl), human applies marker and docblock. No compensating integration tests required.
The function-level docblock must use `//` or `/* */` (per the file's existing comment style) and must contain, at minimum:

- The name of the algorithm or computational pattern (e.g., "recursive-descent parser," "Pratt precedence climbing," "Gauss-Legendre quadrature," "OpenDRIVE XML element dispatch").
- A **justification** for keeping the function as one piece. Valid justifications are specific to this function and include:
  - **Shared state:** internal accumulators, builders, or flags that would need threading through helpers, with no clarity gain.
  - **Profiled performance:** named bottleneck where helper-call overhead matters; cite the profiler output.
  - **Algorithmic cohesion:** the algorithm is a single pass, single recursive descent, or single transaction, and decomposition does not match the algorithm's structure.
  - **Format-driven dispatch:** the branching mirrors a published external format (XODR, JSON, etc.) and splitting would obscure the schema's shape.
- Optional: a reference (textbook, paper, library documentation, format spec) describing the algorithm.

Justifications must explain why no reasonable decomposition exists for the specific function.

The NOLINTNEXTLINE marker goes on the function's declaration or definition line, immediately above the function signature, on the same line as the docblock.


Example:

```cpp
/// \brief Recursive-descent parser for OpenDRIVE lateral profile elements
/// \details Algorithm-essential complexity: each <superelevation>, <shape>,
/// and <crossSectionSurface> branch is a distinct production in the XODR grammar
/// (ASAM OpenDRIVE 1.9 §7.5). Shared state: the active profile pointer and
/// coefficient vector are mutated inline; threading them through helpers would
/// triple the parameter list without clarity gain.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto ParseLateralProfile(pugi::xml_node lat_prof_node) -> ast::LateralProfile;
```

## 3. Done

**Done when:**
- **File mode:** `<path>` is clean (or marked HITL/wedged), committed, and build/tests pass.
- **Directory mode:** Every row in `diagnostic-sweep.md` is ✅ or ⚠️ and a summary has been printed: N clean, M HITL.
