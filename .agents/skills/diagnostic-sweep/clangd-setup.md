# clangd setup troubleshooting

Only reach here when `diagnostics` via `xd://lsp` returns errors.

## Common fixes

- **compile_commands.json missing**: run `cmake --preset dev-debug` (presets set `CMAKE_EXPORT_COMPILE_COMMANDS=ON`). For manual CMake calls, add `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
- **clangd not installed**: install via package manager (`apt install clangd`, `brew install clangd`). Verify with `clangd --version`.
- **Wrong project root**: ensure shell CWD is at the repository root containing `compile_commands.json` and `.clangd`.
- **Server not started or stuck**: run `{"action":"reload","file":"*"}` via `xd://lsp` to restart servers, then re-diagnose.

## Verify

`{"action":"status"}` via `xd://lsp` should show a running server. Then `{"action":"diagnostics","file":"<any .cpp>"}` should return findings (possibly empty) without an error frame.
