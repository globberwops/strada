#include <gtest/gtest.h>

#include <cmath>

#include "../../src/cpm/rotation.hpp"

namespace strada::cpm {

TEST(RotationTest, FromEulerAndToEuler) {
  // Arrange
  double heading = 0.5;
  double pitch = 0.2;
  double roll = -0.1;

  // Act
  auto rot = Rotation::FromEuler(heading, pitch, roll);
  auto euler = rot.ToEuler();

  // Assert
  EXPECT_NEAR(euler.heading, heading, 1e-9);
  EXPECT_NEAR(euler.pitch, pitch, 1e-9);
  EXPECT_NEAR(euler.roll, roll, 1e-9);
}

TEST(RotationTest, TransformAndInverseTransform) {
  // Arrange
  auto rot = Rotation::FromEuler(0.5, 0.2, -0.1);
  double x = 1.0;
  double y = -2.0;
  double z = 3.0;

  // Act
  auto transformed = rot.Transform(x, y, z);
  auto roundtrip = rot.InverseTransform(transformed[0], transformed[1], transformed[2]);

  // Assert
  EXPECT_NEAR(roundtrip[0], x, 1e-9);
  EXPECT_NEAR(roundtrip[1], y, 1e-9);
  EXPECT_NEAR(roundtrip[2], z, 1e-9);
}

TEST(RotationTest, ComposeAndInverse) {
  // Arrange
  auto rot_a = Rotation::FromEuler(0.5, 0.0, 0.0);
  auto rot_b = Rotation::FromEuler(0.0, 0.2, 0.0);

  // Act
  auto rot_composed = rot_a.Compose(rot_b);
  auto rot_inverse = rot_composed.Inverse();
  auto identity = rot_composed.Compose(rot_inverse);
  auto euler_identity = identity.ToEuler();

  // Assert
  EXPECT_NEAR(euler_identity.heading, 0.0, 1e-9);
  EXPECT_NEAR(euler_identity.pitch, 0.0, 1e-9);
  EXPECT_NEAR(euler_identity.roll, 0.0, 1e-9);
}

}  // namespace strada::cpm
