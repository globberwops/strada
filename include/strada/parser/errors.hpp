// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace strada::parser {

/// Base class for all Strada parser errors.
class ParseError : public std::runtime_error {
 public:
  explicit ParseError(std::string_view message) : std::runtime_error(std::string(message)) {}
};

/// Thrown when the XML document is syntactically malformed (pugixml failure).
class XmlParseError : public ParseError {
 public:
  explicit XmlParseError(std::string_view message) : ParseError(message) {}
};

/// Thrown when a mandatory element or attribute is missing from a valid XML document.
class MissingElementError : public ParseError {
 public:
  explicit MissingElementError(std::string_view message) : ParseError(message) {}
};

/// Thrown when an element contains an invalid or unsupported attribute value.
class InvalidAttributeError : public ParseError {
 public:
  explicit InvalidAttributeError(std::string_view message) : ParseError(message) {}
};

}  // namespace strada::parser

