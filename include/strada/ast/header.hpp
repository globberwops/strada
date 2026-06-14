#ifndef STRADA_AST_HEADER_HPP_
#define STRADA_AST_HEADER_HPP_

#include <string>

namespace strada::ast {

struct Header {
  int rev_major_{};
  int rev_minor_{};
  std::string name_;
  std::string version_;
  std::string date_;
  double north_{};
  double south_{};
  double east_{};
  double west_{};
  std::string vendor_;
  std::string geo_reference_;
};

}  // namespace strada::ast

#endif  // STRADA_AST_HEADER_HPP_
