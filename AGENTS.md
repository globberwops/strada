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

### Coding & Testing Conventions

* **Naming Conventions**: Follow the **Google C++ Style Guide**. Type names are `CamelCase` (e.g., `GeometryRecord`), function/method names are `CamelCase` starting with a capital (e.g., `ParseString`), ordinary variables/parameters are `snake_case` (e.g., `xml_content`), and enum values (constants) are `kPascalCase` (e.g., `kNormalized`). For data members, classes must use a trailing underscore (e.g., `name_`), whereas structs must not use a trailing underscore (e.g., `name`).
* **Header Guards**: Use `#pragma once` instead of traditional preprocessor include guards (`#ifndef`) in all header files.
* **Member Initializers**: Initialize fundamental/primitive types (e.g., `int`, `double`, `bool`) with `{}` (e.g., `double length{};`) to avoid uninitialized values. Standard class types (like `std::string`, `std::vector`) must be default-initialized without `{}` to prevent `readability-redundant-member-init` warnings.
* **Test Design**: Follow the **Arrange-Act-Assert (AAA)** pattern in all unit and integration tests. Clearly separate and label these blocks.
* **Testing & Benchmarks**: Use **GoogleTest / GoogleMock** for TDD and **Google Benchmark** for performance regressions.
* **Commit Strategy**: Create exactly **one commit per issue** following the Conventional Commits specification.
* **CMake & Dependencies**: `FetchContent` must try to find the respective system package first using `FIND_PACKAGE_ARGS` (e.g., `FIND_PACKAGE_ARGS NAMES GTest`).
* **Devcontainer**: All library dependencies must be pre-installed in the devcontainer `Dockerfile`.
* **Formatting & Linting**: All changes must pass `clang-format` and `clang-tidy` checks. Avoid `NOLINT` comments unless absolutely unavoidable.
