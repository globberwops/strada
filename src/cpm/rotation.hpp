#pragma once

#include <array>
#include <cmath>
#include <numbers>

namespace strada::cpm {

struct EulerAngles {
  double heading{};
  double pitch{};
  double roll{};
};

class Rotation {
 public:
  constexpr Rotation() = default;

  static auto FromEuler(double heading, double pitch, double roll) noexcept -> Rotation {
    Rotation r;
    r.matrix_ = EulerToMatrix(heading, pitch, roll);
    return r;
  }

  [[nodiscard]] auto ToEuler() const noexcept -> EulerAngles { return MatrixToEuler(matrix_); }

  [[nodiscard]] constexpr auto Compose(const Rotation& other) const noexcept -> Rotation {
    Rotation r;
    r.matrix_ = ComposeRotations(matrix_, other.matrix_);
    return r;
  }

  [[nodiscard]] constexpr auto Inverse() const noexcept -> Rotation {
    Rotation r;
    r.matrix_[0][0] = matrix_[0][0];
    r.matrix_[0][1] = matrix_[1][0];
    r.matrix_[0][2] = matrix_[2][0];
    r.matrix_[1][0] = matrix_[0][1];
    r.matrix_[1][1] = matrix_[1][1];
    r.matrix_[1][2] = matrix_[2][1];
    r.matrix_[2][0] = matrix_[0][2];
    r.matrix_[2][1] = matrix_[1][2];
    r.matrix_[2][2] = matrix_[2][2];
    return r;
  }

  [[nodiscard]] constexpr auto Transform(double x, double y, double z) const noexcept -> std::array<double, 3> {
    return {(matrix_[0][0] * x) + (matrix_[0][1] * y) + (matrix_[0][2] * z),
            (matrix_[1][0] * x) + (matrix_[1][1] * y) + (matrix_[1][2] * z),
            (matrix_[2][0] * x) + (matrix_[2][1] * y) + (matrix_[2][2] * z)};
  }

  [[nodiscard]] constexpr auto InverseTransform(double x, double y, double z) const noexcept -> std::array<double, 3> {
    return {(matrix_[0][0] * x) + (matrix_[1][0] * y) + (matrix_[2][0] * z),
            (matrix_[0][1] * x) + (matrix_[1][1] * y) + (matrix_[2][1] * z),
            (matrix_[0][2] * x) + (matrix_[1][2] * y) + (matrix_[2][2] * z)};
  }

 private:
  using Matrix3x3 = std::array<std::array<double, 3>, 3>;

  static auto EulerToMatrix(double heading, double pitch, double roll) noexcept -> Matrix3x3 {
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);
    const double cos_pitch = std::cos(pitch);
    const double sin_pitch = std::sin(pitch);
    const double cos_roll = std::cos(roll);
    const double sin_roll = std::sin(roll);

    Matrix3x3 rot_matrix;
    rot_matrix[0][0] = cos_heading * cos_pitch;
    rot_matrix[0][1] = ((cos_heading * sin_pitch) * sin_roll) - (sin_heading * cos_roll);
    rot_matrix[0][2] = ((cos_heading * sin_pitch) * cos_roll) + (sin_heading * sin_roll);

    rot_matrix[1][0] = sin_heading * cos_pitch;
    rot_matrix[1][1] = ((sin_heading * sin_pitch) * sin_roll) + (cos_heading * cos_roll);
    rot_matrix[1][2] = ((sin_heading * sin_pitch) * cos_roll) - (cos_heading * sin_roll);

    rot_matrix[2][0] = -sin_pitch;
    rot_matrix[2][1] = cos_pitch * sin_roll;
    rot_matrix[2][2] = cos_pitch * cos_roll;

    return rot_matrix;
  }

  static auto MatrixToEuler(const Matrix3x3& rot_matrix) noexcept -> EulerAngles {
    EulerAngles euler;
    const double sin_pitch = -rot_matrix[2][0];
    constexpr double pi_div2 = 0.5 * std::numbers::pi;
    constexpr double clip_limit = 1.0;
    constexpr double gimbal_lock_threshold = 0.9999999;

    if (sin_pitch <= -clip_limit) {
      euler.pitch = -pi_div2;
    } else if (sin_pitch >= clip_limit) {
      euler.pitch = pi_div2;
    } else {
      euler.pitch = std::asin(sin_pitch);
    }

    // Gimbal lock check
    if (std::abs(rot_matrix[2][0]) >= gimbal_lock_threshold) {
      euler.roll = 0.0;
      if (rot_matrix[2][0] > 0.0) {
        euler.heading = std::atan2(-rot_matrix[0][1], -rot_matrix[0][2]);
      } else {
        euler.heading = std::atan2(rot_matrix[0][1], rot_matrix[0][2]);
      }
    } else {
      euler.roll = std::atan2(rot_matrix[2][1], rot_matrix[2][2]);
      euler.heading = std::atan2(rot_matrix[1][0], rot_matrix[0][0]);
    }

    return euler;
  }

  static constexpr auto ComposeRotations(const Matrix3x3& rot_a, const Matrix3x3& rot_b) noexcept -> Matrix3x3 {
    Matrix3x3 rot_c;
    rot_c[0][0] = (rot_a[0][0] * rot_b[0][0]) + (rot_a[0][1] * rot_b[1][0]) + (rot_a[0][2] * rot_b[2][0]);
    rot_c[0][1] = (rot_a[0][0] * rot_b[0][1]) + (rot_a[0][1] * rot_b[1][1]) + (rot_a[0][2] * rot_b[2][1]);
    rot_c[0][2] = (rot_a[0][0] * rot_b[0][2]) + (rot_a[0][1] * rot_b[1][2]) + (rot_a[0][2] * rot_b[2][2]);

    rot_c[1][0] = (rot_a[1][0] * rot_b[0][0]) + (rot_a[1][1] * rot_b[1][0]) + (rot_a[1][2] * rot_b[2][0]);
    rot_c[1][1] = (rot_a[1][0] * rot_b[0][1]) + (rot_a[1][1] * rot_b[1][1]) + (rot_a[1][2] * rot_b[2][1]);
    rot_c[1][2] = (rot_a[1][0] * rot_b[0][2]) + (rot_a[1][1] * rot_b[1][2]) + (rot_a[1][2] * rot_b[2][2]);

    rot_c[2][0] = (rot_a[2][0] * rot_b[0][0]) + (rot_a[2][1] * rot_b[1][0]) + (rot_a[2][2] * rot_b[2][0]);
    rot_c[2][1] = (rot_a[2][0] * rot_b[0][1]) + (rot_a[2][1] * rot_b[1][1]) + (rot_a[2][2] * rot_b[2][1]);
    rot_c[2][2] = (rot_a[2][0] * rot_b[0][2]) + (rot_a[2][1] * rot_b[1][2]) + (rot_a[2][2] * rot_b[2][2]);
    return rot_c;
  }

  Matrix3x3 matrix_{};
};

}  // namespace strada::cpm
