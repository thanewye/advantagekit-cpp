// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>
#include <vector>

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/util/Alert.hpp>

#include "generated/TunerConstants.h"
#include "subsystems/drive/ModuleIO.h"

class Module {
public:
    using ModuleConstants = swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>;

    Module(std::unique_ptr<ModuleIO> io, int index, const ModuleConstants& constants);

    void Periodic();

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    void RunSetpoint(wpi::math::SwerveModuleVelocity& state);

    /** Runs the module with the specified output while controlling to zero degrees. */
    void RunCharacterization(double output);

    /** Disables all outputs to motors. */
    void Stop();

    /** Returns the current turn angle of the module. */
    wpi::math::Rotation2d GetAngle() const;

    /** Returns the current drive position of the module in meters. */
    double GetPositionMeters() const;

    /** Returns the current drive velocity of the module in meters per second. */
    double GetVelocityMetersPerSec() const;

    /** Returns the module position (turn angle and drive position). */
    wpi::math::SwerveModulePosition GetPosition() const;

    /** Returns the module state (turn angle and drive velocity). */
    wpi::math::SwerveModuleVelocity GetState() const;

    /** Returns the module positions received this cycle. */
    const std::vector<wpi::math::SwerveModulePosition>& GetOdometryPositions() const;

    /** Returns the timestamps of the samples received this cycle. */
    const std::vector<double>& GetOdometryTimestamps() const;

    /** Returns the module position in radians. */
    double GetWheelRadiusCharacterizationPosition() const;

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    double GetFFCharacterizationVelocity() const;

private:
    std::unique_ptr<ModuleIO> io_;
    ModuleIOInputs inputs_;
    int index_;
    ModuleConstants constants_;

    wpi::util::Alert driveDisconnectedAlert_;
    wpi::util::Alert turnDisconnectedAlert_;
    wpi::util::Alert turnEncoderDisconnectedAlert_;
    std::vector<wpi::math::SwerveModulePosition> odometryPositions_;
};
