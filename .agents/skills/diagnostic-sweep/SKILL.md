---
name: diagnostic-sweep
description: Sweep the codebase file-by-file: clean clangd warnings/errors, fresh subagent per row, commit per file.
disable-model-invocation: true
---

# Sweep

Make every file _clean_ — zero clangd findings, build green, tests green, committed. The list IS the state: every row ends ✅ or ⚠️, no row falls through.

**Channel: clangd via `xd://lsp` exclusively.** `clang-tidy` is too slow per file. There is no fallback — if `xd://lsp` errors and the reload/retry path doesn't recover, the sweep stops.

- Schema, response shape, ordering rules, failure modes → [lsp-tool.md](lsp-tool.md)
- Setup troubleshooting → [clangd-setup.md](clangd-setup.md)
- Per-row subagent task template → [subagent-prompt.md](subagent-prompt.md)

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

## 2. Loop

Sequential — one fresh subagent per row, wait, then the next.

Spawn a fresh subagent (no shared context) using the template at [subagent-prompt.md](subagent-prompt.md), parameterized with the row's `#` and `file`. The subagent reads [lsp-tool.md](lsp-tool.md) for the LSP schema and recovery path.

When the subagent returns, verify the row's status and the commit. Pick the next ⬜ row. If the subagent reports `xd://lsp` errors that don't recover, the sweep stops — fix the setup and restart.

## 3. Done

**Done when:** every row is ✅ or ⚠️ and a summary has been printed: N clean, M HITL.
