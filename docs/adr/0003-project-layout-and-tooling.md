# ADR 0003: Project Layout and Tooling

## Status
Accepted

## Context
As we begin implementing Strada, we need to establish standard project layouts and toolchains to support Test-Driven Development (TDD), performance regression testing, code style quality, and layout standardization.

## Decisions

### 1. Pitchfork Layout
We will adopt the standard **Pitchfork Layout** for the repository structure:
* `include/strada/`: Contains public library API headers.
* `src/`: Contains private headers and source files (`.cpp`).
* `tests/`: Contains test suites and test resources.
* `benchmark/`: Contains micro-benchmarks and performance test suites.

Example structure:
```
strada/
├── CMakeLists.txt
├── include/
│   └── strada/
│       ├── ast/
│       └── parser/
├── src/
│   ├── ast/
│   └── parser/
├── tests/
│   └── ast/
└── benchmark/
    └── physics/
```

### 2. Testing Framework: GoogleTest and GoogleMock
* **GoogleTest (gtest)** will be used for all unit and integration tests.
* **GoogleMock (gmock)** will be used to mock dependencies where necessary.
* Tests will be executed via CMake `ctest`.
* We will use CMake `FetchContent` to download and build GoogleTest/GoogleMock automatically at compile time.

### 3. Performance Benchmarking: Google Benchmark
* **Google Benchmark** will be used to measure performance of hot math operations, BVH spatial indexing queries, and parser execution times.
* Benchmarks will live under the `benchmark/` directory.
* We will use CMake `FetchContent` to integrate Google Benchmark automatically.

### 4. Code Quality & Formatting
* **ClangFormat**: Code formatting will be strictly enforced using the existing `.clang-format` configuration. All files must pass format checks before merging.
* **ClangTidy**: Static analysis checks will be enforced using the existing `.clang-tidy` rules to capture code smells, modern C++ warnings, and potential bugs.

## Consequences
* **Standardized Layout**: Follows standard C++ community guidelines, making it easy for third-party CMake projects to consume Strada.
* **TDD Enabled**: Developers can write tests in `tests/` and iterate locally in a test-first loop.
* **Performance Regression Protection**: Benchmarking suites allow measuring the latency impact of code changes, protecting the 1000Hz vehicle dynamics budget.
* **Consistent Quality**: Enforced linting and formatting keep the codebase clean, uniform, and less prone to style debates.
