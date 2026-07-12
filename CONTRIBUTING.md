# Strada Contribution Guide

This guide details the conventions and development guidelines for contributing to the Strada project.

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
* **Commit Strategy**: Create exactly **one commit per issue** following the Conventional Commits specification (squash commits before merging if necessary).
* **CMake & Dependencies**: `FetchContent` must try to find the respective system package first using `FIND_PACKAGE_ARGS` (e.g., `FIND_PACKAGE_ARGS NAMES GTest`).
* **Pre-commit Hooks**: The repository uses `pre-commit` hooks for automatic style enforcement, formatting, and linting (including `clang-format`, `cmake-format`, `cmake-lint`, `shfmt`, `shellcheck`, `check-license-headers`, `gitleaks`, and cached `cppcheck`). These run automatically on `git commit` to ensure rapid feedback during local development.
* **Devcontainer**: All library dependencies must be pre-installed in the devcontainer `Dockerfile`.

### Local Validation Scripts
To easily run the exact same checks locally that are executed on CI, several utility scripts are provided in the [tools/ci/](file:///workspaces/strada/tools/ci/) directory:
*   [run_static_checks.sh](file:///workspaces/strada/tools/ci/run_static_checks.sh): Configures the project under the `dev` preset, runs all pre-commit checks on all files, and runs deep static analysis (`clang-tidy` and `include-what-you-use`) on changed files.
*   [build_and_test.sh <preset>](file:///workspaces/strada/tools/ci/build_and_test.sh): Configures, compiles, and runs tests for a specific CMake preset (`default-debug`, `default-release`, `san-debug`, or `cov-debug`).
*   [generate_coverage.sh](file:///workspaces/strada/tools/ci/generate_coverage.sh): Collects test coverage data and generates an HTML report under `build/cov/coverage_report/index.html`.
*   [run_all.sh](file:///workspaces/strada/tools/ci/run_all.sh): Runs all validation stages sequentially.

Individual tool-specific wrappers (which accept a list of files as arguments, or scan the codebase if none are provided):
*   [run_clang_tidy.sh](file:///workspaces/strada/tools/ci/run_clang_tidy.sh): Runs clang-tidy using the `dev` compilation database.
*   [run_cppcheck.sh](file:///workspaces/strada/tools/ci/run_cppcheck.sh): Runs cppcheck using the `dev` compilation database.
*   [run_iwyu.sh](file:///workspaces/strada/tools/ci/run_iwyu.sh): Runs include-what-you-use check.
