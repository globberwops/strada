# Strada Agent Guidelines

This file provides instructions and configuration for agentic coding assistants working on the Strada project.

## Agent skills

### Issue tracker

Issues are tracked on GitHub, and external PRs are included in triage. See `docs/agents/issue-tracker.md`.

### Triage labels

Triage roles are mapped to default labels (needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix). See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout with one CONTEXT.md and docs/adr/ at the repository root. See `docs/agents/domain.md`.


## Development Guidelines

See [CONTRIBUTING.md](CONTRIBUTING.md) for the project's commit conventions, C++ style and testing guidelines, and build/tooling instructions.

<!-- CODEGRAPH_START -->
## CodeGraph

In repositories indexed by CodeGraph (a `.codegraph/` directory exists at the repo root), reach for it BEFORE grep/find or reading files when you need to understand or locate code:

- **MCP tool** (when available): `codegraph_explore` answers most code questions in one call — the relevant symbols' verbatim source plus the call paths between them, including dynamic-dispatch hops grep can't follow. Name a file or symbol in the query to read its current line-numbered source. If it's listed but deferred, load it by name via tool search.
- **Shell** (always works): `codegraph explore "<symbol names or question>"` prints the same output.

If there is no `.codegraph/` directory, skip CodeGraph entirely — indexing is the user's decision.
<!-- CODEGRAPH_END -->
