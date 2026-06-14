# Strada Agent Guidelines

This file provides instructions and configuration for agentic coding assistants working on the Strada project.

## Agent skills

### Issue tracker

Issues and PRDs for this repo live as GitHub issues. See `docs/agents/issue-tracker.md`.

### Triage labels

Triage roles are mapped to default labels (needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix). See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout with one CONTEXT.md and docs/adr/ at the repository root. See `docs/agents/domain.md`.

## Development Guidelines

### Commit Message Conventions
All commit messages in this repository must follow the **Conventional Commits** specification:

Format: `<type>(<scope>): <description>`

Common types:
* `feat`: A new feature (e.g., parsing a new OpenDRIVE element)
* `fix`: A bug fix (e.g., resolving a parser crash or geometry parsing error)
* `docs`: Documentation changes (e.g., modifying CONTEXT.md or ADRs)
* `style`: Code style modifications (e.g., running clang-format)
* `refactor`: Code reorganization with no functional impact
* `test`: Adding or updating tests (e.g., GoogleTest suites)
* `chore`: Build or tooling updates (e.g., editing CMakeLists.txt)
