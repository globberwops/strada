#pragma once

#include <strada/ast/extensions.hpp>
#include <strada/ast/objects.hpp>
#include <string>
#include <vector>

namespace strada::ast {

/// Represents a dependency of a traffic signal on another signal (t_road_signals_signal_dependency).
struct SignalDependency {
  std::string id;    ///< ID of the dependent signal.
  std::string type;  ///< Optional dependency type, can be empty.
};

/// Represents a reference element associated with a signal (t_road_signals_signal_reference).
struct SignalReferenceElement {
  std::string element_type;  ///< Type of the referenced element (elementType).
  std::string element_id;    ///< ID of the referenced element (elementId).
};

/// Represents a traffic signal inside the road network (t_road_signals_signal).
struct Signal {
  std::string id;                                  ///< Unique ID of the signal.
  std::string name;                                ///< Human-readable name of the signal.
  double s{};                                      ///< Longitudinal coordinate s along reference line.
  double t{};                                      ///< Lateral coordinate t from reference line.
  double z_offset{};                               ///< Height offset relative to road level (zOffset).
  double h_offset{};                               ///< Heading offset relative to road heading (hOffset).
  double roll{};                                   ///< Roll angle.
  double pitch{};                                  ///< Pitch angle.
  Orientation orientation{Orientation::kNone};     ///< Validity direction (none/plus/minus).
  bool dynamic{false};                             ///< Whether the signal is dynamic (yes/no).
  std::string country;                             ///< Country code.
  std::string country_revision;                    ///< Country revision version (countryRevision).
  std::string type;                                ///< Main category/type of the signal.
  std::string subtype;                             ///< Subtype classification of the signal.
  double value{};                                  ///< Numerical value (e.g. speed limit).
  std::string unit;                                ///< Unit of the value (e.g. km/h).
  double height{};                                 ///< Visual height of the signal.
  double width{};                                  ///< Visual width of the signal.
  std::string text;                                ///< Additional text on the signal.
  bool temporary{false};                           ///< Whether the signal is temporary.
  bool invalidated{false};                         ///< Whether the signal is currently invalidated.
  std::vector<SignalDependency> dependencies;      ///< Dependencies of this signal.
  std::vector<SignalReferenceElement> references;  ///< Element references of this signal.
  std::vector<LaneValidity> validities;            ///< Lane validity restrictions.
  Extensions extensions;                           ///< Custom user data extensions.
};

/// Represents a signal reference placed along the road (t_road_signals_signalReference).
struct SignalReference {
  std::string id;                               ///< ID of the referenced signal.
  double s{};                                   ///< Longitudinal coordinate s along reference line.
  double t{};                                   ///< Lateral coordinate t from reference line.
  double z_offset{};                            ///< Height offset relative to road level (zOffset).
  Orientation orientation{Orientation::kNone};  ///< Validity direction (none/plus/minus).
  std::vector<LaneValidity> validities;         ///< Lane validity restrictions.
  Extensions extensions;                        ///< Custom user data extensions.
};

}  // namespace strada::ast
