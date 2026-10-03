// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <cmath>
#include <numbers>
#include <vector>

#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/util/array.hpp>

namespace DriveConstants {
    inline constexpr double maxSpeedMetersPerSec = 4.8;
    inline constexpr double odometryFrequency = 100.0; // Hz
    inline constexpr double trackWidth = wpi::units::meter_t{26.5_in}.value();
    inline constexpr double wheelBase = wpi::units::meter_t{26.5_in}.value();
    inline const double driveBaseRadius = std::hypot(trackWidth / 2.0, wheelBase / 2.0);
    inline constexpr wpi::util::array<wpi::math::Translation2d, 4> moduleTranslations{
        wpi::math::Translation2d{wpi::units::meter_t{trackWidth / 2.0}, wpi::units::meter_t{wheelBase / 2.0}},
        wpi::math::Translation2d{wpi::units::meter_t{trackWidth / 2.0}, wpi::units::meter_t{-wheelBase / 2.0}},
        wpi::math::Translation2d{wpi::units::meter_t{-trackWidth / 2.0}, wpi::units::meter_t{wheelBase / 2.0}},
        wpi::math::Translation2d{wpi::units::meter_t{-trackWidth / 2.0}, wpi::units::meter_t{-wheelBase / 2.0}}};

    // Zeroed rotation values for each module, see setup instructions
    inline constexpr wpi::math::Rotation2d frontLeftZeroRotation{0_rad};
    inline constexpr wpi::math::Rotation2d frontRightZeroRotation{0_rad};
    inline constexpr wpi::math::Rotation2d backLeftZeroRotation{0_rad};
    inline constexpr wpi::math::Rotation2d backRightZeroRotation{0_rad};

    // Device CAN IDs
    inline constexpr int pigeonCanId = 9;

    inline constexpr int frontLeftDriveCanId = 1;
    inline constexpr int backLeftDriveCanId = 3;
    inline constexpr int frontRightDriveCanId = 5;
    inline constexpr int backRightDriveCanId = 7;

    inline constexpr int frontLeftTurnCanId = 2;
    inline constexpr int backLeftTurnCanId = 4;
    inline constexpr int frontRightTurnCanId = 6;
    inline constexpr int backRightTurnCanId = 8;

    // Drive motor configuration
    inline constexpr int driveMotorCurrentLimit = 50;
    inline constexpr double wheelRadiusMeters = wpi::units::meter_t{1.5_in}.value();
    inline constexpr double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth
    // and 22 spur teeth
    inline constexpr wpi::math::DCMotor driveGearbox = wpi::math::DCMotor::NeoVortex(1);

    // Drive encoder configuration
    inline constexpr double driveEncoderPositionFactor = 2 * std::numbers::pi / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    inline constexpr double driveEncoderVelocityFactor = (2 * std::numbers::pi) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Drive PID configuration
    inline constexpr double driveKp = 0.0;
    inline constexpr double driveKd = 0.0;
    inline constexpr double driveKs = 0.0;
    inline constexpr double driveKv = 0.1;
    inline constexpr double driveSimP = 0.05;
    inline constexpr double driveSimD = 0.0;
    inline constexpr double driveSimKs = 0.0;
    inline constexpr double driveSimKv = 0.0789;

    // Turn motor configuration
    inline constexpr bool turnInverted = false;
    inline constexpr int turnMotorCurrentLimit = 20;
    inline constexpr double turnMotorReduction = 9424.0 / 203.0;
    inline constexpr wpi::math::DCMotor turnGearbox = wpi::math::DCMotor::NEO550(1);

    // Turn encoder configuration
    inline constexpr bool turnEncoderInverted = true;
    inline constexpr double turnEncoderPositionFactor = 2 * std::numbers::pi; // Rotations -> Radians
    inline constexpr double turnEncoderVelocityFactor = 2 * std::numbers::pi; // Rotations/Sec -> Rad/Sec

    // Turn PID configuration
    inline constexpr double turnKp = 2.0;
    inline constexpr double turnKd = 0.0;
    inline constexpr double turnSimP = 8.0;
    inline constexpr double turnSimD = 0.0;
    inline constexpr double turnPIDMinInput = 0;                    // Radians
    inline constexpr double turnPIDMaxInput = 2 * std::numbers::pi; // Radians

    // PathPlanner configuration
    inline constexpr double robotMassKg = 74.088;
    inline constexpr double robotMOI = 6.883;
    inline constexpr double wheelCOF = 1.2;
    inline const pathplanner::RobotConfig ppConfig{
        wpi::units::kilogram_t{robotMassKg}, wpi::units::kilogram_square_meter_t{robotMOI},
        pathplanner::ModuleConfig{wpi::units::meter_t{wheelRadiusMeters}, wpi::units::meters_per_second_t{maxSpeedMetersPerSec}, wheelCOF,
                                  wpi::math::DCMotor{driveGearbox}.WithReduction(driveMotorReduction), wpi::units::ampere_t{driveMotorCurrentLimit}, 1},
        std::vector<wpi::math::Translation2d>(moduleTranslations.begin(), moduleTranslations.end())};
} // namespace DriveConstants
