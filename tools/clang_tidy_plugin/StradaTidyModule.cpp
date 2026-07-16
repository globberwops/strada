#include "LocalAutoStyleCheck.h"
#include "clang-tidy/ClangTidy.h"
#include "clang-tidy/ClangTidyModule.h"
#include "clang-tidy/ClangTidyModuleRegistry.h"

namespace clang::tidy::strada {

class StradaTidyModule : public ClangTidyModule {
 public:
  void addCheckFactories(ClangTidyCheckFactories& CheckFactories) override {
    CheckFactories.registerCheck<LocalAutoStyleCheck>("strada-local-auto-style");
  }
};

static ClangTidyModuleRegistry::Add<StradaTidyModule> X("strada-module", "Adds Strada custom clang-tidy checks.");

volatile int StradaTidyModuleAnchorSource = 0;

}  // namespace clang::tidy::strada
