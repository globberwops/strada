# clangd setup troubleshooting

Only reach here if `lsp diagnostics` errors in step 1.

## Common fixes

- **compile_commands.json missing**: configure the project to export it. For CMake presets, the `default` and `dev` presets already set `CMAKE_EXPORT_COMPILE_COMMANDS=ON` — just run `cmake --preset dev-debug`. Otherwise add `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to the configure command.
- **clangd not installed**: install it (`apt install clangd`, `brew install clangd`). Confirm with `clangd --version`.
- **Wrong project root**: `compile_commands.json` and `.clangd` should sit at the project root. `lsp` figures this out from CWD — make sure your shell is at the repo root before invoking the skill.
- **Server not started or stuck**: `lsp reload *` to restart servers, then re-run `lsp diagnostics`.

## Verify

`{"action":"status"}` via `xd://lsp` should show a running server. Then `{"action":"diagnostics","file":"<any .cpp>"}` should return findings (possibly empty) without an error frame.
