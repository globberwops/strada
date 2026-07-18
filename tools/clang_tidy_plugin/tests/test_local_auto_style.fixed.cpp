// Verification test cases for strada-local-auto-style check.

#include <string>
#include <vector>

// Dummy functions to simulate type returns
auto GetLength() -> double { return 42.0; }
auto GetSize() -> int { return 10; }
auto GetString() -> std::string { return "hello"; }

struct Widget {
  explicit Widget(int x) : val(x) {}
  Widget() = default;
  int val{0};
};

auto GetWidget() -> Widget { return Widget{1}; }

// Simple C++20 Concept
template <typename T>
concept Integral = __is_integral(T);

auto GetCount() -> int { return 5; }

void TestCompliant() {
  // Auto-to-Track: variables initialized via functions/complex expressions
  auto length = GetLength();
  const auto size = GetSize();
  const auto& str = GetString();
  auto w = GetWidget();

  // Auto-to-Stick: commit to a type by writing the type name on RHS with '='
  auto name = std::string{"Arthur"};
  auto value = 42.0;
  auto limit = 1.0F;
  auto vec = std::vector<int>{1, 2, 3};
  auto widget = Widget{42};

  // Traditional loop counters inside ForStmt init
  for (int i = 0; i < 10; ++i) {
    // Loop body
  }
  for (auto i = 0U; i < 10U; ++i) {
    // Loop body
  }

  // Range-based loop variables
  auto numbers = std::vector<int>{1, 2, 3};
  for (int x : numbers) {
    (void)x;
  }

  // Concept-constrained variables
  Integral auto count = GetCount();
  const Integral auto const_count = GetCount();

  // Uninitialized variables (exempt)
  int x;
  double y;
  Widget w_uninit;

  // Default-constructed class instances (exempt)
  Widget w_default;
  std::string s_default;

  (void)length;
  (void)size;
  (void)str;
  (void)w;
  (void)name;
  (void)value;
  (void)limit;
  (void)vec;
  (void)widget;
  (void)count;
  (void)const_count;
  (void)x;
  (void)y;
  (void)w_uninit;
  (void)w_default;
  (void)s_default;
}

void TestNonCompliant() {
  // 1. Explicit types initialized from a function (or expression)
  auto length = GetLength();
  const auto size = GetSize();
  const auto& str = GetString();
  auto w = GetWidget();

  // 2. Direct/braced initialization of explicit type
  auto name = std::string{"Arthur"};
  auto value = 42.0;
  auto limit = 1.0F;
  auto count = 5U;
  auto value_stick_init = 42.0;
  auto widget = Widget(42);

  // 3. Braced initialization with '=' (e.g. std::vector<int> v = {1, 2, 3};)
  auto vec = std::vector<int>{1, 2, 3};

  // 4. Direct initialization using auto without '='
  auto auto_name{"Arthur"};
  auto auto_value{42.0};

  (void)length;
  (void)size;
  (void)str;
  (void)w;
  (void)name;
  (void)value;
  (void)limit;
  (void)count;
  (void)value_stick_init;
  (void)widget;
  (void)vec;
  (void)auto_name;
  (void)auto_value;
}
