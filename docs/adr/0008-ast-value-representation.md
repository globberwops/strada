# ADR 0008: AST Value Representation & Validation

## Status
Accepted

## Context
The ASAM OpenDRIVE XML schema permits many optional elements, attributes, and implicit defaults, which if parsed loosely can lead to developer confusion, uninitialized states, or silent failures. To guarantee 100% type correctness and strict alignment with the specification, we need a clear, unified pattern for representing values and closed sets in the AST.

## Decision
We decided to enforce strict value representation rules across the C++ AST and parser layer:
1. **No Implicit Absence**: We forbid representing the absence of an attribute or element using implicit empty values (e.g. empty strings, `0.0`, or `-1`).
2. **Explicit Optionality (`std::optional`)**: True optional fields that have no spec-mandated default behavior (and whose absence means "no value") must be represented as `std::optional<T>` in C++ and default to `std::nullopt`.
3. **Spec-Mandated Defaults**: Fields that have a spec-defined default must be treated as non-optional in C++, default-initialized in the struct definition to that default (e.g. `LayerType layer{LayerType::kPermanent}`), and explicitly assigned that default value by the parser if omitted in the XML.
4. **Strictly Closed Scoped Enums (`enum class`)**: Closed sets of values must be represented as scoped enums (`enum class`) without generic sentinel values like `kUnknown` or `kOther`. Unrecognized strings parsed from XML must immediately throw `InvalidAttributeError`, and missing mandatory fields must throw `MissingElementError`.

## Consequences
- **Type Safety**: The AST's data types exactly reflect the semantic rules of OpenDRIVE, eliminating ambiguity about whether a value was present in the XML.
- **Fail-Fast Parser**: The parser rejects malformed or invalid files early during loading, preventing corrupt states from propagating to downstream layers like the Compiled Physics Model.
