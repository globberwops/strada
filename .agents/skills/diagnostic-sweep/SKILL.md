---
name: diagnostic-sweep
description: Walk every source/header file once, fix clangd warnings/errors per file with a fresh subagent, commit per file.
disable-model-invocation: true
---

# Sweep

Make every file _clean_ — zero clangd diagnostics, build and tests green, committed. The list IS the state: every _row_ ends ✅ or ⚠️, no row falls through.

> **Diagnostics channel: clangd via LSP, exclusively.** clang-tidy is too slow per file — never invoke it during the sweep. Every `lsp diagnostics` call below means clangd.

## 1. Bootstrap

Once, before the loop.

1. **Build the checklist.** Discover the main project's source and header files. Glob recursively from the project root for `*.c *.cc *.cpp *.cxx *.h *.hpp *.hxx *.inl`. Skip build dirs, `tools/`, third-party, generated. Write the table to `diagnostic-sweep.md` at the repo root:

   ```
   | # | File | Status | Note |
   |---|------|--------|------|
   | 1 | src/foo.cpp | ⬜ |  |
   ```

   ⬜ pending, ✅ clean, ⚠️ HITL.

2. **Confirm clangd.** Run `lsp diagnostics` on the first ⬜ file. If it errors, fix the setup (compile_commands.json? clangd installed? wrong root?) before continuing — see [clangd-setup.md](clangd-setup.md) only if stuck.

**Completion criterion:** `diagnostic-sweep.md` exists with every file marked ⬜, and `lsp diagnostics` returns real results on a sample file.

## 2. Loop

For each ⬜ row, _sequentially_ — one _fresh_ subagent, wait for it to finish, then the next:

Spawn one fresh subagent (`agent: "task"`, no shared context) with these instructions:

---

**Task: clean `<file>` (row N)**

Channel: **clangd via LSP** — never run clang-tidy (too slow per file).

You own row N. The focal file is `<file>`, but you may modify other files if a clean fix requires it. Do NOT touch any other row in `diagnostic-sweep.md`.

1. `lsp diagnostics` for `<file>` (clangd). Read each finding.
2. For each diagnostic:
   - Read the cited code.
   - Fix it. The goal is a _clean_ file — style nits included.
   - HITL only when the fix is genuinely ambiguous, breaks a public API, or needs a design call. When in doubt, FIX it.
3. Re-run diagnostics. Repeat until zero findings.
4. `cmake --build --preset dev-debug && ctest --preset dev-debug` (configure first if no build dir: `cmake --preset dev-debug`). All must pass.
5. `git add` the touched files and commit: `diagnostic-sweep: clean <file>`.
6. Edit `diagnostic-sweep.md`: row N → ✅. If HITL, → ⚠️ with a one-line Note explaining what blocked.

**Completion criterion:** row N is ✅ (or ⚠️ with note), the commit exists, build and tests passed.

---

When the subagent returns, verify the row status and the commit. Pick the next ⬜ row.

## 3. Done

When every row is ✅ or ⚠️, print a summary: N clean, M HITL. The _sweep_ is _complete_.
