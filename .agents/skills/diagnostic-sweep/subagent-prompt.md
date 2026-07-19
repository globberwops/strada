# Subagent task template: clean row N

Channel and tool: see [lsp-tool.md](lsp-tool.md) for channel policy, schema, response shape, ordering rules, and the failure-mode recovery path. Read it before starting. `xd://lsp` is the only channel — there is no fallback. If `xd://lsp` errors and recovery doesn't restore it, mark the row ⚠️ with a Note that the tool is wedged; the sweep stops.

You own row N. The focal file is `<file>`, but you may modify other files if a clean fix requires it. Edit only row N in `diagnostic-sweep.md`.

1. `{"action":"diagnostics","file":"<file>"}` via `xd://lsp`. Read each finding.
2. For each diagnostic:
   - Read the cited code.
   - Fix it. The goal is a _clean_ file — style nits included.
   - For renames, use `{"action":"rename","file":...,"line":...,"symbol":...,"new_name":...}` (apply by default; `apply:false` previews). Text-replace across the workspace silently drops call-sites.
   - For server-known quick-fixes, try `{"action":"code_actions","file":...,"line":...,"query":...}` before hand-editing.
   - HITL only when the fix is genuinely ambiguous, breaks a public API, or needs a design call. When in doubt, FIX it.
3. Re-run `{"action":"diagnostics","file":"<file>"}`. Repeat until zero findings.
4. `cmake --build --preset dev-debug && ctest --preset dev-debug` (configure first if no build dir: `cmake --preset dev-debug`). All must pass.
5. `git add` the touched files and commit: `diagnostic-sweep: clean <file>`.
6. Edit `diagnostic-sweep.md`: row N → ✅. If HITL, → ⚠️ with a one-line Note explaining what blocked. If `xd://lsp` is wedged and recovery fails, → ⚠️ with a Note that the tool is wedged; the sweep will stop.

**Done when:** row N is ✅ (or ⚠️ with note), the commit exists, build and tests passed — or row N is ⚠️ because the tool is wedged.
