#pragma once

#include <stdexcept>
#include <string>

namespace strada::parser {

/// Base class for all Strada parser errors.
class ParseError : public std::runtime_error {
 public:
  explicit ParseError(const std::string& message) : std::runtime_error(message) {}
};

/// Thrown when the XML document is syntactically malformed (pugixml failure).
class XmlParseError : public ParseError {
 public:
  explicit XmlParseError(const std::string& message) : ParseError(message) {}
};

/// Thrown when a mandatory element or attribute is missing from a valid XML document.
class MissingElementError : public ParseError {
 public:
  explicit MissingElementError(const std::string& message) : ParseError(message) {}
};

/// Thrown when an element contains an invalid or unsupported attribute value.
class InvalidAttributeError : public ParseError {
 public:
  explicit InvalidAttributeError(const std::string& message) : ParseError(message) {}
};

}  // namespace strada::parser
