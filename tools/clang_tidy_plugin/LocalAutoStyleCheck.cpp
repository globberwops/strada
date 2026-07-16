#include "LocalAutoStyleCheck.h"

#include "clang/AST/ASTContext.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"

using namespace clang::ast_matchers;

namespace clang::tidy::strada {

void LocalAutoStyleCheck::registerMatchers(MatchFinder* Finder) {
  Finder->addMatcher(varDecl(hasLocalStorage(), unless(parmVarDecl())).bind("var"), this);
}

static bool isLoopCounter(const VarDecl* Var, ASTContext& Context) {
  auto Parents = Context.getParents(*Var);
  if (Parents.empty()) {
    return false;
  }
  const auto* DS = Parents[0].get<DeclStmt>();
  if (!DS) {
    return false;
  }
  auto DSParents = Context.getParents(*DS);
  if (DSParents.empty()) {
    return false;
  }
  const auto* FS = DSParents[0].get<ForStmt>();
  if (!FS) {
    return false;
  }
  return FS->getInit() == DS;
}

void LocalAutoStyleCheck::check(const MatchFinder::MatchResult& Result) {
  const auto* MatchedDecl = Result.Nodes.getNodeAs<VarDecl>("var");
  if (!MatchedDecl) {
    return;
  }

  // Ignore macro expansions.
  if (MatchedDecl->getLocation().isMacroID()) {
    return;
  }

  // Exempt uninitialized variables.
  if (!MatchedDecl->hasInit()) {
    return;
  }

  // Exempt traditional loop counters inside ForStmt.
  if (isLoopCounter(MatchedDecl, *Result.Context)) {
    return;
  }

  // Exempt default-constructed class/struct instances with no explicit initializer.
  if (MatchedDecl->getInitStyle() == VarDecl::CallInit) {
    if (const Expr* Init = MatchedDecl->getInit()) {
      const Expr* SubInit = Init;
      while (SubInit) {
        if (const auto* EWC = dyn_cast<ExprWithCleanups>(SubInit)) {
          SubInit = EWC->getSubExpr();
        } else if (const auto* MTE = dyn_cast<MaterializeTemporaryExpr>(SubInit)) {
          SubInit = MTE->getSubExpr();
        } else if (const auto* ICE = dyn_cast<ImplicitCastExpr>(SubInit)) {
          SubInit = ICE->getSubExpr();
        } else {
          break;
        }
      }
      if (const auto* Construct = dyn_cast<CXXConstructExpr>(SubInit)) {
        if (Construct->getParenOrBraceRange().isInvalid()) {
          return;
        }
      }
    }
  }

  // Check if type is explicit (does not contain auto).
  QualType Type = MatchedDecl->getType();
  if (Type.isNull()) {
    return;
  }

  if (Type->getContainedAutoType()) {
    return;
  }

  diag(MatchedDecl->getLocation(), "use auto type deduction for initialized local variables");
}

}  // namespace clang::tidy::strada
