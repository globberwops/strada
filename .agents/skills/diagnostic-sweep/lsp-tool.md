# clangd via `xd://lsp`

Reference for the `xd://lsp` tool surface, response format, ordering rules, and failure recovery.

## Tool surface

| `action` | What it does | Required args | Useful optional args |
|---|---|---|---|
| `diagnostics` | Every current finding (errors + warnings) for a file. **Workhorse — call before fixing, after fixing, and to confirm clean.** | `file` | `timeout` (default 20s; 5–300) |
| `diagnostics` (workspace) | Scan a glob | `file: "*"` | `query` |
| `hover` | Type + doc for a symbol at a position | `file`, `line` (1-indexed), `symbol` (substring) | |
| `definition` | Where a symbol is defined | `file`, `line`, `symbol` | |
| `type_definition` | Where a symbol's type is defined | `file`, `line`, `symbol` | |
| `references` | All call/uses sites of a symbol | `file`, `line`, `symbol` | `apply` (default true; `false` previews) |
| `implementation` | Implementations of a virtual | `file`, `line`, `symbol` | |
| `code_actions` | Server-known quick-fixes / refactors at a position (add include, modernize, NOLINT) | `file`, `line` | `query` (title substring or index); `apply: true` to apply one |
| `rename` | Project-aware symbol rename — never `sed` the workspace | `file`, `line`, `symbol`, `new_name` | `apply: false` to preview, `timeout` |
| `rename_file` | Move a file and rewrite every import / reference | `file`, `new_name` | `apply` |
| `symbols` | File outline; workspace search with `query` | `file` (or `file: "*"` + `query`) | `query` |
| `request` | Raw LSP method escape hatch | `query` = method, `payload` = JSON params | `timeout` |
| `reload` | Restart a server (`file` = one file) or all (`file: "*"`) | `file` | |
| `status` | Server health | | |
| `capabilities` | Server feature list | | |

## Calling pattern

The tool returns the LSP payload verbatim, plus a one-line error frame on failure.

**Diagnostics on a file:**
```
{"action":"diagnostics","file":"src/cpm/bounding_volume_hierarchy.cpp"}
```

**Symbol references before a rename:**
```
{"action":"references","file":"include/strada/cpm/coordinate.hpp","line":12,"symbol":"Pose"}
```

**Code action (auto-fix a modernize loop, or `NOLINT` add):**
```
{"action":"code_actions","file":"src/foo.cpp","line":42,"query":"modernize"}
```

## Reading the response

For `action: "diagnostics"` the payload is structured as:

```
{N} error(s), {M} warning(s):
# <directory>/
## <file>
  <line>:<col> [error|warning] [clang-tidy] <message> (<check-name>)
  ...
```

- `severity: error` — blocking; the build fails with `-Werror`.
- `severity: warning` — clang-tidy finding (style, modernization, perf, readability). All warnings count; style nits are real.
- `<check-name>` (in parens) is the rule id; look up the rule's intent when the fix is non-obvious.
- A return of `OK` (no diagnostic count line) means the file is already clean.

## Ordering rules

Do these in order, every row.

1. Diagnose first, then fix. Trust `lsp diagnostics <file>` over file reading — it returns the exact line:col and check id for every finding.
2. Re-verify after every edit. The LSP server may re-publish diagnostics; the only proof of a clean file is another `lsp diagnostics` returning zero findings.
3. Use `code_actions` before hand-editing for: adding an include, modernizing a loop/cast, adding a `NOLINT`, renaming a symbol. The server knows the safe fix.
4. Use `rename` (not text replace) for any symbol rename. Text-replace across the workspace silently drops call-sites.
5. Use `references` before editing an exported symbol. Missed call-sites are bugs, not style.
6. Reload only when stuck. `{"action":"reload","file":"*"}` restarts every server; do this if diagnostics errors out or the server seems wedged.

## Failure modes

Each error has a recovery action; if recovery fails, the file is _wedged_ — mark ⚠️ and stop.

- `ERROR: server not running` → `{"action":"status"}`. If empty, `{"action":"reload","file":"*"}` and retry. If still broken, the sweep stops.
- `ERROR: no compile_commands.json` → `cmake --preset dev-debug` (presets already set `CMAKE_EXPORT_COMPILE_COMMANDS=ON`) and retry. If still broken, the sweep stops.
- `ERROR: file not in compilation database` → the file is not in any CMake target. Check the directory's `CMakeLists.txt`; some headers are only included by tests or examples. Reconfigure and retry.
- clangd reports findings on a header the parent `CMakeLists.txt` didn't glob → add it to the source list, reconfigure, re-diagnose.
- Two consecutive `xd://lsp` errors on the same file, after a `reload`, → the sweep stops. Do not retry indefinitely.
