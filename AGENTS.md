# Strada Agent Guidelines

This file provides instructions and configuration for agentic coding assistants working on the Strada project.

## Agent skills

### Issue tracker

Issues are tracked on GitHub, and external PRs are included in triage. See `docs/agents/issue-tracker.md`.

### Triage labels

Triage roles are mapped to default labels (needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix). See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout with one CONTEXT.md and docs/adr/ at the repository root. See `docs/agents/domain.md`.

### Subagent Spawning via Herdr

When spawning subagents (e.g. for parallel research, task execution, or code reviews), agents must always prefer using the `herdr` terminal workspace manager to launch new `agy` instances in split panes, rather than relying on built-in subagent tool calls.

To check if you are currently running inside a `herdr` workspace, verify if the following environment variable is defined and set:
- `HERDR_ENV=1` (or if `HERDR_SOCKET_PATH` is present).

To keep the agent relationship tree visually clear from left to right:
- **Sequential agents** (subsequent tasks in a workflow) must split the current panel vertically (`--split right`), allowing progress to flow left-to-right.
- **Parallel agents** (simultaneous sub-tasks) must split the panel vertically (`--split right`) for the first instance, and then split subsequent sibling tasks horizontally (`--split down`) to stack them vertically under that step.

To launch a new subagent in a split pane to the right, run:
```bash
herdr agent start "<agent-name>" --split right -- agy -i "/<command>"
```

**Herd Concurrency Control**: Before launching a new group ("herd") of subagents, you must verify if any previously spawned subagents from the prior task are still active. If the previous herd has not finished running, you must refuse to spawn a new one until they complete.


## Development Guidelines

See [CONTRIBUTING.md](CONTRIBUTING.md) for the project's commit conventions, C++ style and testing guidelines, and build/tooling instructions.
