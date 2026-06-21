// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/ids.hpp>
#include <string>
#include <vector>

namespace strada::tess {

/// Represents a 3D vertex position using 32-bit floats for GPU rendering efficiency.
struct Vertex {
  float x{};  ///< Cartesian X coordinate (meters).
  float y{};  ///< Cartesian Y coordinate (meters).
  float z{};  ///< Cartesian Z coordinate (meters).
};

/// Represents a static index and vertex buffer mesh of a lane or road surface.
struct Mesh {
  std::vector<Vertex> vertices;   ///< Flat list of 3D vertices.
  std::vector<uint32_t> indices;  ///< Indices defining triangle layout.
  cpm::RoadId road_id{};          ///< Compiled ID of the parent road.
  cpm::LaneId lane_id{};          ///< Compiled ID of the lane.
  std::string lane_type;          ///< The structural type of the lane (e.g., "driving", "sidewalk").
};

/// Represents a static set of points forming a line boundary or marking.
struct Polyline {
  std::vector<Vertex> vertices;   ///< Consecutive 3D points forming the line.
  cpm::RoadId road_id{};          ///< Compiled ID of the parent road.
  int original_lane_id{0};        ///< The original index ID from the XODR source (e.g. -1, 1, 0).
  bool is_reference_line{false};  ///< True if this polyline represents the road's center reference line.
  std::string marking_type;       ///< Styling description of the marking (e.g., "solid", "broken", "none").
};

/// Holds the output layers of tessellated road network geometries.
class Tessellator {
 public:
  /// Constructs the Tessellator layer from the Logical AST.
  ///
  /// Internally instantiates a temporary CompiledPhysicsModel to evaluate geometries
  /// with the specified chord error tolerance.
  ///
  /// \param map The logical C++ abstract syntax tree of the map.
  /// \param chord_error The maximum chord error tolerance in meters (e.g., 0.5m).
  Tessellator(const ast::AbstractSyntaxTree& map, double chord_error);

  /// Default constructor.
  Tessellator() = default;

  /// Destructor.
  ~Tessellator() = default;

  /// Returns the flat list of generated road surface meshes.
  [[nodiscard]] auto Meshes() const noexcept -> const std::vector<Mesh>& { return meshes_; }

  /// Returns the flat list of generated boundary polylines.
  [[nodiscard]] auto Polylines() const noexcept -> const std::vector<Polyline>& { return polylines_; }

 private:
  std::vector<Mesh> meshes_;
  std::vector<Polyline> polylines_;
};

}  // namespace strada::tess
