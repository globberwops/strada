# Strada Agent Guidelines

This file provides instructions and configuration for agentic coding assistants working on the Strada project.

## Agent skills

### Issue tracker

Issues and PRDs for this repo live as GitHub issues. External PRs are included in triage. See `docs/agents/issue-tracker.md`.

### Triage labels

Triage roles are mapped to default labels (needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix). See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout with one CONTEXT.md and docs/adr/ at the repository root. See `docs/agents/domain.md`.

### Idea backlog

Rough ideas and feature proposals that need to be fleshed out before entering the main specification flow live in the Wayfinder Map on the issue tracker (see [Strada Idea Backlog Wayfinder Map](https://github.com/globberwops/strada/issues/56)).


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

### Coding & Testing Conventions

* **C++ Style & Coding Standards**: Follow the project's [C++ Style Reference](docs/guidelines/cpp_style.md), which details naming, formatting, non-owning views, trailing return types, Doxygen documentation, member initialization, and formatting/linting compliance.
* **Test Design**: Follow the **Arrange-Act-Assert (AAA)** pattern in all unit and integration tests. Clearly separate and label these blocks.
* **Testing & Benchmarks**: Use **GoogleTest / GoogleMock** for TDD and **Google Benchmark** for performance regressions.
* **Commit Strategy**: Create exactly **one commit per issue** following the Conventional Commits specification.
* **CMake & Dependencies**: `FetchContent` must try to find the respective system package first using `FIND_PACKAGE_ARGS` (e.g., `FIND_PACKAGE_ARGS NAMES GTest`).
* **Devcontainer**: All library dependencies must be pre-installed in the devcontainer `Dockerfile`.
* **Pre-commit Hooks**: The repository uses `pre-commit` hooks for automatic style enforcement, formatting, and linting (including `clang-format`, `clang-tidy`, `cppcheck`, `cpplint`, `include-what-you-use`, and `gitleaks`). To ensure commit hooks pass successfully, run formatting and linting tools locally (e.g., `pre-commit run --files <files>`) before staging and committing changes.
