// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <strada/cpm/ids.hpp>

namespace strada::cpm {

/// Represents a 3D position and orientation pose in the global inertial reference frame.
struct InertialPose {
  double x{};        ///< Global Cartesian x coordinate (meters).
  double y{};        ///< Global Cartesian y coordinate (meters).
  double z{};        ///< Global Cartesian z coordinate (meters).
  double heading{};  ///< Heading/yaw orientation angle (radians).
  double pitch{};    ///< Pitch orientation angle (radians).
  double roll{};     ///< Roll orientation angle (radians).
};

/// Represents a 3D position and orientation pose in a road-local reference frame.
struct RoadPose {
  double s{};        ///< Station s-coordinate along the road's reference line (meters).
  double t{};        ///< Lateral t-coordinate offset from the reference line (meters).
  double h{};        ///< Height h-coordinate offset from the road surface (meters).
  double heading{};  ///< Road-relative heading angle (radians).
  double pitch{};    ///< Road-relative pitch angle (radians).
  double roll{};     ///< Road-relative roll angle (radians).
  RoadId road{};     ///< Compiled unique ID of the road.
};

/// Represents a 3D position and orientation pose in a lane-local reference frame.
struct LanePose {
  double s{};        ///< Station s-coordinate along the parent road's reference line (meters).
  double t{};        ///< Lateral t-coordinate offset from the lane's center line (meters).
  double h{};        ///< Height h-coordinate offset from the lane surface (meters).
  double heading{};  ///< Lane-relative heading angle (radians).
  double pitch{};    ///< Lane-relative pitch angle (radians).
  double roll{};     ///< Lane-relative roll angle (radians).
  RoadId road{};     ///< Compiled unique ID of the parent road.
  LaneId lane{};     ///< Compiled unique ID of the lane.
};

/// Represents a 3D point in the global inertial reference frame.
struct InertialPosition {
  double x{};  ///< Global Cartesian x coordinate (meters).
  double y{};  ///< Global Cartesian y coordinate (meters).
  double z{};  ///< Global Cartesian z coordinate (meters).
};

}  // namespace strada::cpm
