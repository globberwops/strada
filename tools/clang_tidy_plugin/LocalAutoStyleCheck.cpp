#include "LocalAutoStyleCheck.h"

#include "clang/AST/ASTContext.h"
#include "clang/AST/Expr.h"
#include "clang/AST/ExprCXX.h"
#include "clang/AST/StmtCXX.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"
#include "clang/Lex/Lexer.h"

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
  if (const auto* FS = DSParents[0].get<ForStmt>()) {
    return FS->getInit() == DS;
  }
  if (const auto* CRS = DSParents[0].get<CXXForRangeStmt>()) {
    return true;
  }
  return false;
}

static bool isWhitespaceBefore(SourceLocation Loc, const SourceManager& SM) {
  if (Loc.isInvalid()) {
    return false;
  }
  bool Invalid = false;
  const char* CharData = SM.getCharacterData(Loc.getLocWithOffset(-1), &Invalid);
  if (Invalid || !CharData) {
    return false;
  }
  char C = *CharData;
  return C == ' ' || C == '\t' || C == '\n' || C == '\r';
}

static std::string stripNumericSuffix(std::string Str) {
  bool IsHex = (Str.size() > 2 && Str[0] == '0' && (Str[1] == 'x' || Str[1] == 'X'));
  bool IsHexFloat = IsHex && (Str.find('.') != std::string::npos || Str.find_first_of("pP") != std::string::npos);

  while (!Str.empty()) {
    char Back = Str.back();
    if (IsHex && !IsHexFloat) {
      if (Back == 'u' || Back == 'U' || Back == 'l' || Back == 'L') {
        Str.pop_back();
      } else {
        break;
      }
    } else {
      if (Back == 'f' || Back == 'F' || Back == 'u' || Back == 'U' || Back == 'l' || Back == 'L') {
        Str.pop_back();
      } else {
        break;
      }
    }
  }
  return Str;
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

  // Exempt compiler-generated implicit declarations.
  if (MatchedDecl->isImplicit()) {
    return;
  }

  // Check type deduction/style
  QualType Type = MatchedDecl->getType();
  if (Type.isNull()) {
    return;
  }

  if (const auto* AT = Type->getContainedAutoType()) {
    // Exempt concept-constrained auto types
    if (AT->isConstrained()) {
      return;
    }
    // Enforce '=' assignment initialization style for auto variables.
    if (MatchedDecl->getInitStyle() != VarDecl::CInit) {
      diag(MatchedDecl->getLocation(), "use '=' assignment syntax for local variables using auto type deduction");
    }
    return;
  }

  // Get unqualified type range
  if (TypeSourceInfo* TSI = MatchedDecl->getTypeSourceInfo()) {
    TypeLoc TL = TSI->getTypeLoc();
    while (true) {
      TL = TL.getUnqualifiedLoc();
      if (auto RefTL = TL.getAs<ReferenceTypeLoc>()) {
        TL = RefTL.getPointeeLoc();
      } else if (auto PtrTL = TL.getAs<PointerTypeLoc>()) {
        TL = PtrTL.getPointeeLoc();
      } else {
        break;
      }
    }
    SourceRange UnqualifiedRange = TL.getSourceRange();
    if (UnqualifiedRange.isValid()) {
      std::string TypeText = Lexer::getSourceText(CharSourceRange::getTokenRange(UnqualifiedRange),
                                                  *Result.SourceManager, Result.Context->getLangOpts())
                                 .str();

      if (!TypeText.empty()) {
        const Expr* Init = MatchedDecl->getInit();
        if (Init) {
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

          // Check if it's a literal or single-element init-list/constructor containing a literal
          const Expr* InnerInit = SubInit;
          while (InnerInit) {
            if (const auto* EWC = dyn_cast<ExprWithCleanups>(InnerInit)) {
              InnerInit = EWC->getSubExpr();
            } else if (const auto* MTE = dyn_cast<MaterializeTemporaryExpr>(InnerInit)) {
              InnerInit = MTE->getSubExpr();
            } else if (const auto* CE = dyn_cast<CastExpr>(InnerInit)) {
              InnerInit = CE->getSubExpr();
            } else if (const auto* ILE = dyn_cast<InitListExpr>(InnerInit)) {
              if (ILE->getNumInits() == 1) {
                InnerInit = ILE->getInit(0);
              } else {
                break;
              }
            } else if (const auto* CCE = dyn_cast<CXXConstructExpr>(InnerInit)) {
              if (CCE->getNumArgs() == 1) {
                InnerInit = CCE->getArg(0);
              } else {
                break;
              }
            } else {
              break;
            }
          }

          bool IsLiteral = false;
          std::string LiteralStr;
          if (const auto* FL = dyn_cast<FloatingLiteral>(InnerInit)) {
            IsLiteral = true;
            SourceRange LitRange = FL->getSourceRange();
            LiteralStr = Lexer::getSourceText(CharSourceRange::getTokenRange(LitRange), *Result.SourceManager,
                                              Result.Context->getLangOpts())
                             .str();
          } else if (const auto* IL = dyn_cast<IntegerLiteral>(InnerInit)) {
            IsLiteral = true;
            SourceRange LitRange = IL->getSourceRange();
            LiteralStr = Lexer::getSourceText(CharSourceRange::getTokenRange(LitRange), *Result.SourceManager,
                                              Result.Context->getLangOpts())
                             .str();
          } else if (const auto* CL = dyn_cast<CharacterLiteral>(InnerInit)) {
            IsLiteral = true;
            SourceRange LitRange = CL->getSourceRange();
            LiteralStr = Lexer::getSourceText(CharSourceRange::getTokenRange(LitRange), *Result.SourceManager,
                                              Result.Context->getLangOpts())
                             .str();
          } else if (const auto* BL = dyn_cast<CXXBoolLiteralExpr>(InnerInit)) {
            IsLiteral = true;
            LiteralStr = BL->getValue() ? "true" : "false";
          }

          const clang::Type* CanonicalType = Type.getCanonicalType().getTypePtr();
          const auto* BT = dyn_cast<BuiltinType>(CanonicalType);

          bool IsBuiltinLiteral = false;
          std::string FormattedLiteral;
          if (IsLiteral && BT) {
            IsBuiltinLiteral = true;
            std::string Cleaned = stripNumericSuffix(LiteralStr);
            std::string Suffix;
            if (BT->getKind() == BuiltinType::Float) {
              Suffix = "F";
            } else if (BT->getKind() == BuiltinType::LongDouble) {
              Suffix = "L";
            } else if (BT->getKind() == BuiltinType::UInt || BT->getKind() == BuiltinType::UShort) {
              Suffix = "U";
            } else if (BT->getKind() == BuiltinType::ULong) {
              Suffix = "UL";
            } else if (BT->getKind() == BuiltinType::ULongLong) {
              Suffix = "ULL";
            } else if (BT->getKind() == BuiltinType::Long) {
              Suffix = "L";
            } else if (BT->getKind() == BuiltinType::LongLong) {
              Suffix = "LL";
            } else if (BT->getKind() == BuiltinType::Double) {
              if (Cleaned.find('.') == std::string::npos && Cleaned.find_first_of("eEpP") == std::string::npos) {
                Suffix = ".0";
              }
            }
            FormattedLiteral = Cleaned + Suffix;
          }

          bool IsListInit = false;
          if (isa<InitListExpr>(SubInit)) {
            IsListInit = true;
          } else if (const auto* Construct = dyn_cast<CXXConstructExpr>(SubInit)) {
            if (Construct->isListInitialization()) {
              IsListInit = true;
            }
          }

          SourceLocation InitStart;
          if (const auto* Construct = dyn_cast<CXXConstructExpr>(SubInit)) {
            InitStart = Construct->getParenOrBraceRange().getBegin();
            if (InitStart.isInvalid()) {
              InitStart = Init->getBeginLoc();
            }
          } else if (const auto* ILE = dyn_cast<InitListExpr>(SubInit)) {
            InitStart = ILE->getBeginLoc();
          } else {
            InitStart = Init->getBeginLoc();
          }

          auto Diagnostic = diag(MatchedDecl->getLocation(), "use auto type deduction for initialized local variables");
          Diagnostic << FixItHint::CreateReplacement(CharSourceRange::getTokenRange(UnqualifiedRange), "auto");

          if (IsBuiltinLiteral) {
            if (MatchedDecl->getInitStyle() == VarDecl::CInit) {
              Diagnostic << FixItHint::CreateReplacement(CharSourceRange::getTokenRange(Init->getSourceRange()),
                                                         FormattedLiteral);
            } else {
              SourceLocation InitEnd;
              if (const auto* Construct = dyn_cast<CXXConstructExpr>(SubInit)) {
                InitEnd = Construct->getParenOrBraceRange().getEnd();
              } else if (const auto* ILE = dyn_cast<InitListExpr>(SubInit)) {
                InitEnd = ILE->getEndLoc();
              }
              if (InitEnd.isInvalid()) {
                InitEnd = Init->getEndLoc();
              }
              if (InitStart.isValid() && InitEnd.isValid()) {
                Diagnostic << FixItHint::CreateReplacement(
                    CharSourceRange::getTokenRange(SourceRange(InitStart, InitEnd)), " = " + FormattedLiteral);
              }
            }
          } else {
            if (MatchedDecl->getInitStyle() == VarDecl::ListInit || MatchedDecl->getInitStyle() == VarDecl::CallInit) {
              // Case 2: direct/braced initialization without = (e.g. std::string name{"Arthur"};)
              if (InitStart.isValid()) {
                Diagnostic << FixItHint::CreateInsertion(InitStart, " = " + TypeText);
              }
            } else if (MatchedDecl->getInitStyle() == VarDecl::CInit && IsListInit) {
              // Case 3: braced initialization with = (e.g. std::vector<int> v = {1, 2, 3};)
              if (InitStart.isValid()) {
                if (isWhitespaceBefore(InitStart, *Result.SourceManager)) {
                  Diagnostic << FixItHint::CreateInsertion(InitStart, TypeText);
                } else {
                  Diagnostic << FixItHint::CreateInsertion(InitStart, " " + TypeText);
                }
              }
            }
          }
          return;
        }
      }
    }
  }

  diag(MatchedDecl->getLocation(), "use auto type deduction for initialized local variables");
}

}  // namespace clang::tidy::strada
