---
name: herdr-spawning
description: herd management and subagent spawning via the herdr workspace manager. Use when spawning subagents, launching parallel research, running sequential tasks, or doing code reviews.
---

# Herd Management with Herdr

A **herd** is a group of related subagents running in split terminal panes managed by the `herdr` workspace manager. Using herds maintains a clear visual relationship tree and prevents resource collisions.

Always prefer spawning new instances of the parent agent via `herdr` terminal commands over built-in subagent tool calls.

## Steps

### 1. Verify Workspace Environment
Check the environment variables before invoking subagents.
- Verify that `HERDR_ENV=1` or `HERDR_SOCKET_PATH` is defined.
* **Completion criterion**: Active environment variables are verified, confirming `herdr` capability.

### 2. Enforce Herd Concurrency
Maintain herd isolation and manage system resources by running one herd at a time.
- Verify if any previously spawned sibling or parent subagents are still active.
- Wait for all active subagents of the prior herd to complete.
- Close all terminal panes from the completed herd.
* **Completion criterion**: All active panes from previous herds are closed.

### 3. Establish Split Layout
Organize the herd visually to represent the workflow topology:
- **Sequential tasks**: For workflows where tasks execute chronologically, split the current panel vertically to the right:
  ```bash
  herdr agent start "<agent-name>" --split right -- <parent-agent-executable> -i "/<command>"
  ```
- **Parallel tasks**: For simultaneous sub-tasks, split the first instance vertically to the right, and stack subsequent sibling tasks horizontally under it:
  ```bash
  # Start the first herd member (split right)
  herdr agent start "<agent-name-1>" --split right -- <parent-agent-executable> -i "/<command>"

  # Stack subsequent herd members under it (split down)
  herdr agent start "<agent-name-2>" --split down -- <parent-agent-executable> -i "/<command>"
  ```
  *(Note: Replace `<parent-agent-executable>` with the executable command of the calling/parent agent, e.g. `agy`.)*
* **Completion criterion**: The target split flags (`--split right` vs `--split down`) match the workflow structure.
