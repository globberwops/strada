#pragma once

#include "clang-tidy/ClangTidyCheck.h"

namespace clang::tidy::strada {

/// Enforces the Strada auto type deduction and local variable initialization style.
///
/// Under this check, initialized local variables (except traditional loop variables)
/// must use auto type deduction. They should also use the '=' operator for initialization.
class LocalAutoStyleCheck : public ClangTidyCheck {
 public:
  LocalAutoStyleCheck(StringRef Name, ClangTidyContext* Context) : ClangTidyCheck(Name, Context) {}

  void registerMatchers(ast_matchers::MatchFinder* Finder) override;
  void check(const ast_matchers::MatchFinder::MatchResult& Result) override;
};

}  // namespace clang::tidy::strada
