// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/ModuleIOSim.h"

#include <algorithm>
#include <cmath>

#include <wpi/math/system/Models.hpp>
#include <wpi/system/Timer.hpp>

ModuleIOSim::ModuleIOSim(
    const swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>& constants)
    // Create drive and turn sim models
    : driveSim_(wpi::math::Models::SingleJointedArmFromPhysicalConstants(DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio), DRIVE_GEARBOX)
    , turnSim_(wpi::math::Models::SingleJointedArmFromPhysicalConstants(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio), TURN_GEARBOX) {
    // Enable wrapping for turn PID
    turnController_.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void ModuleIOSim::UpdateInputs(ModuleIOInputs& inputs) {
    // Run closed-loop control
    if (driveClosedLoop_) {
        driveAppliedVolts_ = driveFFVolts_ + driveController_.Calculate(driveSim_.GetAngularVelocity().value());
    } else {
        driveController_.Reset();
    }
    if (turnClosedLoop_) {
        turnAppliedVolts_ = turnController_.Calculate(turnSim_.GetAngularPosition().value());
    } else {
        turnController_.Reset();
    }

    // Update simulation state
    driveSim_.SetInputVoltage(wpi::units::volt_t{std::clamp(driveAppliedVolts_, -12.0, 12.0)});
    turnSim_.SetInputVoltage(wpi::units::volt_t{std::clamp(turnAppliedVolts_, -12.0, 12.0)});
    driveSim_.Update(20_ms);
    turnSim_.Update(20_ms);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim_.GetAngularPosition().value();
    inputs.driveVelocityRadPerSec = driveSim_.GetAngularVelocity().value();
    inputs.driveAppliedVolts = driveAppliedVolts_;
    inputs.driveCurrentAmps = std::abs(driveSim_.GetCurrentDraw().value());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = wpi::math::Rotation2d{turnSim_.GetAngularPosition()};
    inputs.turnPosition = wpi::math::Rotation2d{turnSim_.GetAngularPosition()};
    inputs.turnVelocityRadPerSec = turnSim_.GetAngularVelocity().value();
    inputs.turnAppliedVolts = turnAppliedVolts_;
    inputs.turnCurrentAmps = std::abs(turnSim_.GetCurrentDraw().value());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = {wpi::Timer::GetTimestamp().value()};
    inputs.odometryDrivePositionsRad = {inputs.drivePositionRad};
    inputs.odometryTurnPositions = {inputs.turnPosition};
}

void ModuleIOSim::SetDriveOpenLoop(double output) {
    driveClosedLoop_ = false;
    driveAppliedVolts_ = output;
}

void ModuleIOSim::SetTurnOpenLoop(double output) {
    turnClosedLoop_ = false;
    turnAppliedVolts_ = output;
}

void ModuleIOSim::SetDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop_ = true;
    driveFFVolts_ = DRIVE_KS * ((velocityRadPerSec > 0) - (velocityRadPerSec < 0)) + DRIVE_KV * velocityRadPerSec;
    driveController_.SetSetpoint(velocityRadPerSec);
}

void ModuleIOSim::SetTurnPosition(const wpi::math::Rotation2d& rotation) {
    turnClosedLoop_ = true;
    turnController_.SetSetpoint(rotation.Radians().value());
}
