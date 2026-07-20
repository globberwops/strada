---
name: diagnostic-sweep
description: Clean clangd findings across a file or directory with per-file commits.
disable-model-invocation: true
---

# Sweep

**Relentless sweep.** Make every file _clean_ — zero clangd findings, build green, tests green, committed.

**Channel: clangd via `xd://lsp` exclusively.** If `xd://lsp` is _wedged_ — two consecutive errors on the same file after a reload — the sweep stops.

- Reference, payload format, failure modes → [lsp-tool.md](lsp-tool.md)
- Setup troubleshooting → [clangd-setup.md](clangd-setup.md)

## 1. Target Handling

Accepts `<path>` (defaults to `.`):

- **File mode (`<path>` is a file):** Process directly through [File Loop](#2-file-processing-loop) without creating `diagnostic-sweep.md`.
- **Directory mode (`<path>` is a directory or `.`):**
  - Glob C/C++ source and header files recursively inside `<path>`: `*.c *.cc *.cpp *.cxx *.h *.hpp *.hxx *.inl`. Exclude build dirs, `tools/`, third-party, generated.
  - Write `diagnostic-sweep.md` at repo root:

    ```markdown
    | # | File | Status | Note |
    |---|------|--------|------|
    | 1 | src/foo.cpp | ⬜ |  |
    ```

    Status markers: ⬜ pending, ✅ clean, 🛑 no unit test coverage, ⚠️ HITL / wedged.
  - Process each row sequentially through [File Loop](#2-file-processing-loop).

## 2. File Processing Loop

Finish each file entirely — diagnose, fix, re-verify, build, test, commit, update status — before proceeding to the next.

1. **Diagnose:** `{"action":"diagnostics","file":"<file>"}` via `xd://lsp`.
2. **Evaluate findings:** For each diagnostic:
   - **Coverage Gate:** Verify unit test coverage for cited code. If uncovered, leave unedited and mark 🛑 (Note: `No unit test coverage for <symbol>`).
   - Query `code_actions` at finding location first; prefer LSP Fix-its and `rename` over hand-editing. Use `references` before editing exported symbols.
   - If unambiguous, fix to achieve a _clean_ file (style nits included).
   - If fix requires design decisions, breaks a public API, or hits magic-number / cognitive-complexity HITL rules, route to ⚠️ HITL (Note: formula/ref or algorithm/justification).
3. **Re-verify:** Re-run `diagnostics`. Repeat step 2 until zero findings remain (or only 🛑/⚠️ findings remain).
4. **Build & Test:** `cmake --build --preset dev-debug && ctest --preset dev-debug`.
5. **Commit:** `git add` touched files and commit: `style(diagnostic-sweep): clean <file>`.
6. **Update Status:** In directory mode, edit `diagnostic-sweep.md` row N → ✅ clean, 🛑 no coverage, ⚠️ HITL/wedged. In file mode, report final status. If wedged, stop the sweep.

### Magic-number handling

For `cppcoreguidelines-avoid-magic-numbers` or `readability-magic-numbers`:

- **Fix (named constant):** Default for domain constants (tolerances, thresholds, physical constants). Extract named constant (e.g. `1e-9` → `kMinCurvature`).
- **Suppress via NOLINT (HITL):** For structural constants (position in formula or series expansion). Route to ⚠️ HITL. Note format: formula name + reference.

```cpp
// Taylor series for sin(x): x - x^3/3! + x^5/5! - x^7/7! + ...
// Reference: Abramowitz & Stegun §4.3
// NOLINTNEXTLINE
const double kSeriesCoef = 5040.0;
```

### Cognitive-complexity handling

For `readability-function-cognitive-complexity`:

- **Refactor (extract helpers):** Default for incidental complexity (tangled logic, sequential blocks). Extract single-responsibility helpers until findings clear.
- **Suppress via NOLINT (HITL):** For algorithmic-essential complexity (state machine, recursive descent, multi-field parser). Route to ⚠️ HITL. Note format: algorithm name + justification.

```cpp
/// \brief Recursive-descent parser for OpenDRIVE lateral profile elements
/// \details Algorithm-essential complexity: each <superelevation>, <shape>,
/// and <crossSectionSurface> branch is a distinct production in the XODR grammar
/// (ASAM OpenDRIVE 1.9 §7.5). Shared state: active profile pointer mutated inline.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto ParseLateralProfile(pugi::xml_node lat_prof_node) -> ast::LateralProfile;
```

## 3. Completion Criteria

**Done when:**
- **File mode:** `<path>` is clean (or marked 🛑 no test coverage / ⚠️ HITL / wedged), committed (if modified), build and tests pass.
- **Directory mode:** Every row in `diagnostic-sweep.md` is ✅, 🛑, or ⚠️ and summary reported (N clean, K no test coverage, M HITL).
