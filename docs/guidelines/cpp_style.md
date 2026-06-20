# Google C++ Style Guide: Strada Agent Reference

This document provides a condensed, highly structured reference of the **Google C++ Style Guide**, refined with **Strada project-specific overrides** to serve as a direct implementation and enforcement guide for AI coding agents.

---

## 1. Strada Project-Specific Overrides (Priority Rules)

The Strada codebase overrides the standard Google C++ Style Guide in several key areas. These rules **must** take precedence over Google defaults:

- **Column Limit:** **120 characters** (instead of 80).
- **Header Guards:** Always use `#pragma once` (instead of `#ifndef` define guards).
- **Curly Braces:** Curly braces `{}` are **strictly required** for all controlled statements (`if`, `else`, `for`, `while`), even single-line bodies.
- **Trailing Return Types:**
  - Functions returning a non-`void` type must use trailing return type syntax (`auto FunctionName(params) -> ReturnType;`) unconditionally.
  - Functions returning `void` must use leading return type syntax (`void FunctionName(params);`).
  - Return type deduction without a trailing arrow (e.g., `auto Function();` or `auto Function() { ... }`) is **forbidden**.
  - Constructors and destructors are excluded from return type annotations.
- **Non-Owning Views:** Prefer passing `std::span<const T>` and `std::string_view` by value instead of `const std::vector<T>&` and `const std::string&` for read-only contiguous sequences and string parameters.
- **Member Initializers:**
  - Initialize fundamental/primitive types (e.g., `int`, `double`, `bool`, pointers) using `{}` (e.g., `double length{};`) to prevent uninitialized values.
  - Standard library and custom class types (e.g., `std::string`, `std::vector`) must be default-initialized **without** `{}` to prevent `clang-tidy` warning `readability-redundant-member-init`.
- **Exceptions & Hot-Paths:** Exceptions **must not** be used in performance-critical hot-paths (e.g., geometry queries, coordinate translations, physics calculations). Exceptions are permitted only in non-hot-paths (e.g., startup file parsing, initialization).
- **Ownership & Smart Pointers:** Never use raw pointers for ownership. Use `std::unique_ptr<T>` for exclusive ownership, and `std::shared_ptr<T>` only for shared ownership. Pass non-owning access using raw pointers `T*` (if nullable) or references `T&` (if non-nullable).
- **Enum Trailing Commas:** Trailing commas in enums must be **removed** (enforced by `clang-format`).
- **End-of-File Newline:** Always insert a newline at the end of every file.
- **Include Sorting & Regrouping:** Sort includes case-sensitively (`SortIncludes: CaseSensitive`) and regroup includes based on block categories (`IncludeBlocks: Regroup`).
- **Linting Compliance:** All code must pass both `clang-format` and `clang-tidy` checks. Avoid `NOLINT` comments unless absolutely unavoidable.

---

## 2. Naming Conventions

The style of a name immediately identifies the category of the entity without looking up its declaration.

| Entity | Casing | Example | Notes |
| :--- | :--- | :--- | :--- |
| **File Names** | `snake_case` | `url_table.cc`, `url_table.h` | All lowercase. Can use dashes if local code dictates. Suffixes: `.cc`, `.h`, `.inc` (textual inclusions). |
| **Type Names** | `CamelCase` | `UrlTable`, `UrlTableProperties` | Classes, structs, type aliases, enums, concepts, type template parameters. |
| **Concept Names** | `CamelCase` | `UrlTableReader` | Matches Type Names. |
| **Variable Names** | `snake_case` | `table_name`, `num_entries` | Local variables, function parameters, and struct data members. |
| **Class Data Members** | `snake_case_` | `table_name_`, `pool_` | Regular member variables in a `class` (never a `struct`). Must have a trailing underscore. |
| **Struct Data Members** | `snake_case` | `table_name`, `pool` | Data members in a `struct`. Do **not** use trailing underscores. |
| **Constant Names** | `kPascalCase` | `kDaysInWeek`, `kAndroidVersion` | Fixed values declared `const` or `constexpr` at global, namespace, or class scope. Optional/discouraged for local function constants. |
| **Function Names** | `PascalCase` | `AddTableEntry()`, `OpenFileOrDie()` | Normal methods and free functions. |
| **Accessors & Mutators** | `snake_case` | `table_name()`, `set_table_name()` | Read/write helpers matching the underlying variable name. |
| **Namespace Names** | `snake_case` | `my_project::my_component` | Lowercase. Nested namespaces must avoid collisions with `std` or other external libraries. |
| **Enumerator Names** | `kPascalCase` | `kOk`, `kOutOfMemory` | Enum values (scoped/unscoped). Do **not** use `ALL_CAPS`. |
| **Template Type Parameters** | `CamelCase` | `typename T`, `typename ElementType` | Matches Type Names. |
| **Template Non-Type Params**| Category-based | `int kSize`, `int size` | Follows variable or constant naming conventions depending on constancy. |
| **Macro Names** | `ALL_CAPS` | `PROJECT_ASSERT(x)` | Avoid macros. If necessary, prefix with project/library name. |

---

## 3. Type Deduction (`auto`) Guidelines

Type deduction should be used to make the code **clearer or safer**, and **never** merely to avoid writing an explicit type. Under Google C++ conventions, type deduction falls into two modern styles: **Auto-to-Track** and **Auto-to-Stick**.

### 3.1 Auto-to-Track
**Auto-to-Track** is using `auto` to deduce a variable's type directly from the return value of a function or expression.

*   **When to Use:**
    *   For complex, long, or template-heavy types (such as iterators: `auto it = my_map.find(key);`).
    *   In structured bindings to avoid map key mismatch errors: `const auto& [key, value] = *it;`.
    *   When the type is fully obvious from the expression itself: `auto widget = std::make_unique<Widget>();`.
*   **When to Avoid:**
    *   If the type of the expression is not immediately obvious from the context: `auto foo = x.add_foo();` (bad if the reader cannot tell what `foo` is).
    *   Avoid using it if there is a risk of unintended copy conversions (prefer `const auto&` to bind by reference).

### 3.2 Auto-to-Stick
**Auto-to-Stick** is using `auto` with the type explicitly written on the right-hand side of the initialization using a constructor, cast, or brace-initializer.

*   **When to Use:**
    *   For objects or custom classes to enforce a clean left-to-right reading pattern and guarantee zero-initialization: `auto options = FrobberOptions{};`.
    *   With explicit casts or factories: `auto error_code = static_cast<int>(FrobberError::kMalformedUrl);`.
*   **When to Avoid:**
    *   **Prohibited for Primitives:** For simple, primitive variables (e.g. `int`, `double`, `bool`, pointers, references), **never** use the Auto-to-Stick format. Declare their types explicitly and use braced initialization: `int retries{5};` (not `auto retries = int{5};`).

---

## 4. Non-Owning View & Optional Parameter Guidelines

In C++20, prefer passing lightweight, non-owning views (`std::span` and `std::string_view`) instead of container or string references for read-only parameters. Additionally, use clear pointer conventions for optional parameters.

### 4.1 `std::span` vs. Container References
Prefer passing **span-likes** (such as `std::span<const T>`) instead of **container const references** (such as `const std::vector<T>&`) for function inputs representing contiguous sequences of elements.

- **Decoupling:** Decouples interfaces from the container implementation. The function accepts `std::vector`, `std::array`, C-style arrays, `std::string`, or custom memory buffers without templates or overloads.
- **Cheap Pass-by-Value:** A `std::span` is a tiny, non-owning view (consisting only of a pointer and a size) that fits in registers. Pass it by value (`std::span<const T>`), not by reference.
- **Slicing Support:** Callers can easily pass sub-ranges (slices) of their containers without allocating or copying memory.
- **When to Still Use References:**
  - If you need to store or persist a reference to the container itself (very rare for `const` references).
  - If the interface specifically requires passing ownership of the container (use `std::vector<T>` by value or move).
  - If you are interoperating with legacy libraries or third-party APIs that strictly require `const std::vector<T>&`.

### 4.2 `std::string_view` vs. `const std::string&`
Prefer passing `std::string_view` by value instead of `const std::string&` for read-only string parameters.

- **Zero Allocation for Literals:** Passing a string literal (e.g. `"hello"`) to a `const std::string&` parameter forces the compiler to construct a temporary `std::string` object, which triggers a dynamic memory allocation. Passing it to a `std::string_view` has zero allocation overhead.
- **Flexibility:** Accepts `std::string`, `std::string_view`, and `const char*` without overload clutter.
- **Pass-by-Value:** Passed by value as a lightweight pointer-size pair.
- **Critical Caveat (When to use `const std::string&`):**
  - **Null-Termination:** `std::string_view` is **not** guaranteed to be null-terminated (e.g. if it is a slice of a larger string). If the function passes the string buffer to a legacy C API requiring a null terminator (such as `fopen`, `open`, or Unix sys-calls), you **must** use `const std::string&` (allowing `.c_str()`) or explicitly null-terminate it.

### 4.3 Optional Parameters Guideline
When passing arguments that may or may not be present, follow these strict rules to prevent unnecessary allocations or copy overhead:
- **Optional inputs of primitive/small types:** Pass by value as `std::optional<T>` (e.g., `std::optional<int> max_count`).
- **Optional inputs of heavy/complex types:** Pass as `const T*` (e.g., `const FrobberOptions* options`). Use `nullptr` to represent absence.
- **Optional outputs or input-output parameters:** Pass as a raw pointer `T*` (e.g., `UrlList* output`). Use `nullptr` to represent absence.

---

## 5. Doxygen Commenting Guidelines

The Strada project uses **Doxygen** to build API documentation. AI agents must document all public interfaces, classes, structs, functions, enums, and variables using standard Doxygen special comment blocks.

### 5.1 Comment block syntax
- Use the triple-slash `///` syntax for documenting C++ code blocks. Do **not** use Qt-style `/*!` or Javadoc-style `/**` comments in new code.
- To document class/struct members, parameters, or enumerators inline (after the member), use the `///<` syntax.

### 5.2 Brief and Detailed Descriptions
- A **brief description** is a short, one-line summary. It starts automatically on the first line of the `///` block and ends at the first punctuation mark (dot, question mark, exclamation) followed by a space.
- A **detailed description** provides more in-depth operational or design information. Separate it from the brief description by an empty line containing only `///`.
- Write in complete, grammatically correct sentences. The implied subject is *"This function"* or *"This class"*, and the description should start with third-person present tense verbs (e.g. `/// Opens the connection.`, not `/// Open the connection.`).

### 5.3 Parameter and Return Documentation
Use standard Doxygen commands inside comment blocks to detail arguments, return values, and cross-references. Prefacing commands with a backslash `\` is preferred:

- `\param` (or `\param[in]`, `\param[out]`, `\param[in,out]`): Describe a function parameter.
- `\return`: Describe the output/return value of the function.
- `\sa`: See Also. Use to cross-reference related classes, functions, or documentation.
- `\warning`, `\note`, `\bug`: strategic callouts for performance implications, threading invariants, or known issues.

### 5.4 Documentation Location
- Always document functions in the **header file** (`.h`) directly above their declaration.
- Internal implementation details (how a function does its work step-by-step) belong in the `.cc` file as standard double-slash `//` comments. Do **not** use Doxygen `///` comments for implementation details.

---

## 6. Formatting Rules

Whitespace and layout consistency ensures automated tooling (e.g., `clang-format`) runs smoothly without conflicts.

- **Line Length:** **120 characters** maximum. Exceptions: URLs, un-wrappable strings/regex, system `#include` statements, and header guards.
- **Indentation:** Use **2 spaces** for normal indentation. Never use tabs.
- **Namespace Indentation:** Code within namespaces is **not** indented.
- **Keyword Indentation:** In classes, `public:`, `protected:`, and `private:` are indented by **1 space**.
- **Blank Lines:**
  - Do not add blank lines at the start or end of blocks (e.g., right after `{` or before `}`).
  - Precede `public:`, `protected:`, and `private:` labels with a blank line (optional in small classes). Do not put a blank line after them.
- **Declarations & Wraps:**
  - Function parameters that do not fit on one line must wrap and be indented by **4 spaces** (or aligned with the first parameter).
  - Preprocessor directives (such as `#if`, `#define`) must start at column 0, even inside indented blocks.
- **Horizontal Whitespace:**
  - Space after keywords (`if`, `while`, `for`, `switch`).
  - No space after opening parent/before closing paren in conditions (e.g., `if (x == y)`).
  - Spaces around binary operators (`=`, `+`, `&&`, etc.).
  - No spaces around member access operators (`.`, `->`).
  - Pointer operators have no space after asterisk or ampersand (`char* c;`, `const std::string& str;`, `*p`). No space between the pointer character and closing template brackets (e.g., `std::vector<char*>`).

---

## 7. C++ Code Showcase

The following is a comprehensive, compile-ready C++ example showcasing all formatting, naming, Doxygen comment, initialization, type deduction, view type (`std::span` and `std::string_view`) usage, trailing and leading return type syntaxes, and structure rules in action.

### Header File: `frobber.h`
```cpp
// Copyright 2026 Google LLC
// SPDX-License-Identifier: Apache-2.0
//
// This file declares the Frobber utility class and its support structs.
// It provides functionality to process system URLs and track active state.

#pragma once  // Strada standard header guard

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace my_project::my_component {

/// Configuration attributes for configuring a UrlFrobber instance.
struct FrobberOptions {
  std::string output_prefix;
  int max_retries{3};  ///< Maximum number of network retries. Primitive initialized with {}
  bool enable_cache{true};  ///< Toggles cache checks. Primitive initialized with {}
};

/// Represents different processing error categories.
enum class FrobberError {
  kOk = 0,
  kNetworkFailure,
  kMalformedUrl,
  kAccessDenied
};  // Note: Trailing comma removed as per Strada formatting rules

/// Processes URL resources and coordinates cache verification.
///
/// This class handles retrieving and caching network payloads.
/// It is NOT thread-safe. Concurrent access must be synchronized by
/// the caller using an external mutex.
///
/// Example:
/// \code
///    FrobberOptions options;
///    options.enable_cache = false;
///    UrlFrobber frobber(options);
///    bool success = frobber.ProcessUrl("https://example.com");
/// \endcode
class UrlFrobber {
 public:
  // Custom type aliases appear first in the public section.
  using UrlList = std::vector<std::string>;

  // Static constants follow.
  static constexpr int kMaxConnections = 100;

  // Constructors & destructors are excluded from return type annotations.
  explicit UrlFrobber(const FrobberOptions& options);
  ~UrlFrobber() = default;

  // Disable copying.
  UrlFrobber(const UrlFrobber&) = delete;
  UrlFrobber& operator=(const UrlFrobber&) = delete;

  // Enable moving.
  UrlFrobber(UrlFrobber&&) noexcept = default;
  UrlFrobber& operator=(UrlFrobber&&) noexcept = default;

  /// Processes the specified URL.
  ///
  /// Connects to the host, verifies cache availability if configured, and
  /// processes the payload.
  ///
  /// \param url The target HTTPS address to retrieve. Passed by string_view.
  /// \return True if processed successfully, false on error.
  /// \sa IsCached
  auto ProcessUrl(std::string_view url) -> bool;

  /// Processes a collection of URLs sequentially.
  ///
  /// Iterates through the provided list of URLs and applies ProcessUrl to each.
  ///
  /// \param urls A view over a contiguous list of target URL strings.
  /// \return True if all URLs successfully processed, false if any process step fails.
  auto ProcessUrls(std::span<const std::string> urls) -> bool;

  /// Resets the internal processed URLs counter to zero.
  ///
  /// Since this function returns void, it must use the leading return type.
  void ResetProcessedCount();

  // Accessors and mutators use trailing return type (returning non-void).
  auto processed_count() const -> int { return processed_count_; }
  auto options() const -> const FrobberOptions& { return options_; }

 private:
  /// Verifies if the target URL exists in the local preload cache.
  auto IsCached(std::string_view url) const -> bool;

  // Data members in a class have a trailing underscore.
  const FrobberOptions options_;  // Class type: Default-initialized without {} to prevent tidy warnings
  int processed_count_{};         // Primitive type: Initialized with {} to avoid uninitialized values
  int last_error_code_{-1};       // Primitive type: Initialized with -1
};

}  // namespace my_project::my_component
```

### Implementation File: `frobber.cc`
```cpp
// Copyright 2026 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "my_project/my_component/frobber.h"

#include <algorithm>
#include <span>
#include <string>
#include <string_view>

namespace my_project::my_component {

namespace {

// Variables in anonymous namespaces have internal linkage.
constexpr char kDefaultProtocol[] = "https://";

// Helper function internal to this source file. Uses trailing return type.
auto HasValidProtocol(std::string_view url) -> bool {
  return url.rfind(kDefaultProtocol, 0) == 0;
}

}  // namespace

UrlFrobber::UrlFrobber(const FrobberOptions& options)
    : options_(options) {}  // processed_count_ default-initialized with {} in the header, no redundant init here

auto UrlFrobber::ProcessUrl(std::string_view url) -> bool {
  // Strada project requires curly braces on all control statements
  if (!HasValidProtocol(url)) {
    // Declared explicitly with type and braced initialization
    int error{static_cast<int>(FrobberError::kMalformedUrl)};
    last_error_code_ = error;
    return false;
  }

  // Example of horizontal spacing and control statements formatting.
  if (IsCached(url) && options_.enable_cache) {
    ++processed_count_;
    return true;
  }

  // Example path demonstrating hot-path non-exception error handling.
  const bool network_success = true;
  if (!network_success) {
    int error{static_cast<int>(FrobberError::kNetworkFailure)};
    last_error_code_ = error;
    return false;
  }

  ++processed_count_;
  return true;
}

auto UrlFrobber::ProcessUrls(std::span<const std::string> urls) -> bool {
  for (const auto& url : urls) {
    // Hot-paths do not use exceptions. We verify boolean return statuses instead.
    if (!ProcessUrl(url)) {
      return false;
    }
  }
  return true;
}

void UrlFrobber::ResetProcessedCount() {
  processed_count_ = 0;
}

auto UrlFrobber::IsCached(std::string_view url) const -> bool {
  // Tricky implementation logic is commented here.
  // We use linear lookup as this collection is expected to remain small.
  static const char* const kPrefetchedUrls[] = {
      "https://example.com/index.html",
      "https://example.com/assets.js",
  };

  for (const char* cached_url : kPrefetchedUrls) {
    if (url == cached_url) {
      return true;
    }
  }

  return false;
}

}  // namespace my_project::my_component
```

---

## 8. C++ Core Guidelines Integration

Strada integrates the C++ Core Guidelines to promote safety, type correctness, and modern resource management. Where conflicts arise between the Core Guidelines and Strada overrides, the **Strada overrides always take precedence**.

### 8.1 Resource Management & Memory Safety
- **No Raw `new` or `delete` (CG R.11):** Avoid raw allocation operators. Use `std::make_unique` for allocating single objects on the heap.
- **Raw Pointers are Non-Owning (CG R.3):** A raw pointer (`T*`) must always represent a non-owning, nullable view. Never call `delete` on a raw pointer.
- **Smart Pointers for Ownership (CG R.20):** Use `std::unique_ptr` for exclusive ownership, and `std::shared_ptr` only when ownership is shared.
- **No GSL Dependency:** Strada does not import the Guideline Support Library (`gsl`). Use standard library types (e.g. `std::span`, standard pointers, and references) instead of GSL vocabulary types like `gsl::owner`, `gsl::not_null`, or `gsl::string_span`.

### 8.2 Const Correctness & State
- **Const by Default (CG Con.1, Con.2, Con.3):** Declare all variables, parameters, and return types `const` or `constexpr` by default unless they must be mutable.
- **Const Member Functions (CG M.2):** Mark member functions `const` if they do not modify the logical state of the object.
- **No `const_cast` (CG ES.50):** Avoid `const_cast` to modify variables. If a variable must change state, it should not be `const`.

### 8.3 Class Design & Invariants
- **Use Class for Invariants (CG C.2):** Define a `class` if the object maintains state invariants (rules governing member combinations). Members must be private.
- **Use Struct for Passive Data (CG C.2):** Define a `struct` if the object is a passive record of data with no invariants. Members must be public.
- **Single-Argument Constructors (CG C.46):** Mark all single-argument constructors as `explicit` to prevent implicit type conversions.

### 8.4 Strada Overrides vs. Core Guidelines Reference Table

| Core Guideline | CG Recommendation | Strada Enforcement | Rationale |
| :--- | :--- | :--- | :--- |
| **Error Handling (CG E.2)** | Throw exceptions for all failures | Exceptions **only in non-hot paths**. Hot-paths return `std::optional` or boolean status | Performance optimization to avoid stack unwinding tables on CPU-intensive queries |
| **Type Initialization (CG ES.23)** | Use `{}` for all unified initialization | Primitive types use `{}` initialization. STL/class types use default constructor (`std::string s;` without `{}`) | Prevents `readability-redundant-member-init` warnings from `clang-tidy` |
| **Function Return Syntax (CG F.21)** | Traditional leading return types (`int f();`) | Trailing return types unconditionally for all non-void functions (`auto f() -> int;`) | Enhanced readability, uniformity, and simplified template signatures |
