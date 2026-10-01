// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>

namespace DriveConstants {
    inline constexpr double maxSpeedMetersPerSec = 4.0;
    inline constexpr double trackWidth = units::meter_t{26.0_in}.value();

    // Device CAN IDs
    inline constexpr int pigeonCanId = 9;
    inline constexpr int leftLeaderCanId = 1;
    inline constexpr int leftFollowerCanId = 2;
    inline constexpr int rightLeaderCanId = 3;
    inline constexpr int rightFollowerCanId = 4;

    // Motor configuration
    inline constexpr int currentLimit = 60;
    inline constexpr double wheelRadiusMeters = units::meter_t{3.0_in}.value();
    inline constexpr double motorReduction = 10.71;
    inline constexpr bool leftInverted = false;
    inline constexpr bool rightInverted = true;
    inline constexpr frc::DCMotor gearbox = frc::DCMotor::CIM(2);

    // Velocity PID configuration
    inline constexpr double realKp = 0.0;
    inline constexpr double realKd = 0.0;
    inline constexpr double realKs = 0.0;
    inline constexpr double realKv = 0.1;

    inline constexpr double simKp = 0.05;
    inline constexpr double simKd = 0.0;
    inline constexpr double simKs = 0.0;
    inline constexpr double simKv = 0.227;

    // PathPlanner configuration
    inline constexpr double robotMassKg = 74.088;
    inline constexpr double robotMOI = 6.883;
    inline constexpr double wheelCOF = 1.2;
    inline const pathplanner::RobotConfig ppConfig{units::kilogram_t{robotMassKg}, units::kilogram_square_meter_t{robotMOI},
                                                   pathplanner::ModuleConfig{units::meter_t{wheelRadiusMeters}, units::meters_per_second_t{maxSpeedMetersPerSec},
                                                                             wheelCOF, frc::DCMotor{gearbox}.WithReduction(motorReduction), units::ampere_t{currentLimit}, 2},
                                                   units::meter_t{trackWidth}};
} // namespace DriveConstants
