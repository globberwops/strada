---
name: diagnostic-sweep
description: Sweep the codebase file-by-file: clean clangd warnings/errors, commit per file.
disable-model-invocation: true
---

# Sweep

Make every file _clean_ — zero clangd findings, build green, tests green, committed. The list IS the state: every row ends ✅ or ⚠️, no row falls through.

**Channel: clangd via `xd://lsp` exclusively.** `clang-tidy` is too slow per file. If `xd://lsp` is _wedged_ — two consecutive errors on the same file after a reload — the sweep stops.

- Schema, response shape, ordering rules, failure modes → [lsp-tool.md](lsp-tool.md)
- Setup troubleshooting → [clangd-setup.md](clangd-setup.md)

## 1. Bootstrap

Once, before the loop.

1. Build the checklist. Glob recursively from the project root: `*.c *.cc *.cpp *.cxx *.h *.hpp *.hxx *.inl`. Skip build dirs, `tools/`, third-party, generated. Write the table to `diagnostic-sweep.md` at the repo root:

   ```
   | # | File | Status | Note |
   |---|------|--------|------|
   | 1 | src/foo.cpp | ⬜ |  |
   ```

   ⬜ pending, ✅ clean, ⚠️ HITL.

2. Confirm clangd. `{"action":"diagnostics","file":"<first .cpp file>"}` via `xd://lsp`. On error, follow the recovery path in [lsp-tool.md](lsp-tool.md) § Failure modes; see [clangd-setup.md](clangd-setup.md) only if stuck.

**Done when:** `diagnostic-sweep.md` lists every source/header marked ⬜, and `xd://lsp diagnostics` returns real results on a sample file.

## 2. Process each row in this session

Work rows sequentially in this session. Finish row N entirely — diagnose, fix, re-verify, build, test, commit, mark — before starting row N+1.

1. `{"action":"diagnostics","file":"<file>"}` via `xd://lsp`. Read each finding.
2. For each diagnostic:
   - Read the cited code.
   - Fix it. The goal is a _clean_ file — style nits included.
   - Prefer `code_actions` and `rename` over hand-editing — see ordering rules in [lsp-tool.md](lsp-tool.md).
   - HITL only when the fix is genuinely ambiguous, breaks a public API, or needs a design call. When in doubt, FIX it.
3. Re-run `{"action":"diagnostics","file":"<file>"}`. Repeat until zero findings.
4. `cmake --build --preset dev-debug && ctest --preset dev-debug` (configure first if no build dir: `cmake --preset dev-debug`). All must pass.
5. `git add` the touched files and commit: `style(diagnostic-sweep): clean <file>`.
6. Edit `diagnostic-sweep.md`: row N → ✅. If HITL, → ⚠️ with a one-line Note. If wedged, → ⚠️ and the sweep stops.

After row N is ✅ or ⚠️, pick the next ⬜ row.

**Done when:** row N is ✅ (or ⚠️ with note), the commit exists, build and tests passed — or row N is ⚠️ because the tool is wedged.

## 3. Done

**Done when:** every row is ✅ or ⚠️ and a summary has been printed: N clean, M HITL.
