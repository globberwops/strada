# clangd via `xd://lsp`

Reference for the `xd://lsp` tool surface, response format, and failure recovery.

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
| `code_actions` | Clang Fix-its and server-known refactors at a position (add include, modernize, NOLINT). **Primary fix path — query before hand-editing.** | `file`, `line` | `query` (title substring or index); `apply: true` to apply one |
| `rename` | Project-aware symbol rename | `file`, `line`, `symbol`, `new_name` | `apply: false` to preview, `timeout` |
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
- `severity: warning` — clang-tidy finding (style, modernization, perf, readability).
- `<check-name>` (in parens) is the rule id; look up the rule's intent when the fix is non-obvious.
- A return of `OK` (no diagnostic count line) means the file is already clean.

## Failure modes & Recovery

Each error has a recovery action; if recovery fails after reload, the file is _wedged_ — mark ⚠️ and stop.

- `ERROR: server not running` → check `{"action":"status"}`. If empty, `{"action":"reload","file":"*"}` and retry.
- `ERROR: no compile_commands.json` → see [clangd-setup.md](clangd-setup.md).
- `ERROR: file not in compilation database` → verify file is listed in target `CMakeLists.txt`, reconfigure (`cmake --preset dev-debug`), and retry.
- Wedged (2 consecutive errors on same file after reload) → mark ⚠️ and stop sweep.
