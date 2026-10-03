// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <vector>

#include <wpi/math/geometry/Rotation2d.hpp>

struct ModuleIOInputs {
    bool driveConnected = false;
    double drivePositionRad = 0.0;
    double driveVelocityRadPerSec = 0.0;
    double driveAppliedVolts = 0.0;
    double driveCurrentAmps = 0.0;

    bool turnConnected = false;
    wpi::math::Rotation2d turnPosition{};
    double turnVelocityRadPerSec = 0.0;
    double turnAppliedVolts = 0.0;
    double turnCurrentAmps = 0.0;

    std::vector<double> odometryTimestamps{};
    std::vector<double> odometryDrivePositionsRad{};
    std::vector<wpi::math::Rotation2d> odometryTurnPositions{};
};

class ModuleIO {
public:
    virtual ~ModuleIO() = default;

    /** Updates the set of loggable inputs. */
    virtual void UpdateInputs(ModuleIOInputs& inputs) {}

    /** Run the drive motor at the specified open loop value. */
    virtual void SetDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    virtual void SetTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    virtual void SetDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    virtual void SetTurnPosition(const wpi::math::Rotation2d& rotation) {}
};
