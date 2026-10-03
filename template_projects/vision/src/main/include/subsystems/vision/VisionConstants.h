// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <array>
#include <limits>
#include <numbers>
#include <string>

#include <wpi/fields/Field.hpp>
#include <wpi/fields/fields.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>

namespace VisionConstants {
    // AprilTag layout
    inline const wpi::fields::Field aprilTagLayout = wpi::fields::GetField(wpi::fields::FieldId::DEFAULT_FIELD);

    // Camera names, must match names configured on coprocessor
    inline const std::string camera0Name = "camera_0";
    inline const std::string camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    inline const wpi::math::Transform3d robotToCamera0{0.2_m, 0.0_m, 0.2_m, wpi::math::Rotation3d{0.0_rad, -0.4_rad, 0.0_rad}};
    inline const wpi::math::Transform3d robotToCamera1{-0.2_m, 0.0_m, 0.2_m, wpi::math::Rotation3d{0.0_rad, -0.4_rad, wpi::units::radian_t{std::numbers::pi}}};

    // Basic filtering thresholds
    inline double maxAmbiguity = 0.3;
    inline double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    inline double linearStdDevBaseline = 0.02;  // Meters
    inline double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    inline std::array<double, 2> cameraStdDevFactors{
        1.0, // Camera 0
        1.0  // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    inline double linearStdDevMegatag2Factor = 0.5;                                      // More stable than full 3D solve
    inline double angularStdDevMegatag2Factor = std::numeric_limits<double>::infinity(); // No rotation data available
} // namespace VisionConstants
