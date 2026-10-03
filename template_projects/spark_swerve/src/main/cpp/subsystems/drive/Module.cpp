// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/Module.h"

#include <string>

#include <akit/Logger.h>

#include "subsystems/drive/DriveConstants.h"

using namespace DriveConstants;

Module::Module(std::unique_ptr<ModuleIO> io, int index)
    : io_(std::move(io))
    , index_(index)
    , driveDisconnectedAlert_("Drive/Module" + std::to_string(index) + "/DriveDisconnected",
                              "Disconnected drive motor on module " + std::to_string(index) + ".", wpi::util::Alert::Level::HIGH)
    , turnDisconnectedAlert_("Drive/Module" + std::to_string(index) + "/TurnDisconnected", "Disconnected turn motor on module " + std::to_string(index) + ".",
                             wpi::util::Alert::Level::HIGH) {}

void Module::Periodic() {
    io_->UpdateInputs(inputs_);
    akit::Logger::ProcessInputs("Drive/Module" + std::to_string(index_), inputs_);

    // Calculate positions for odometry
    size_t sampleCount = inputs_.odometryTimestamps.size(); // All signals are sampled together
    odometryPositions_.resize(sampleCount);
    for (size_t i = 0; i < sampleCount; i++) {
        double positionMeters = inputs_.odometryDrivePositionsRad[i] * wheelRadiusMeters;
        wpi::math::Rotation2d angle = inputs_.odometryTurnPositions[i];
        odometryPositions_[i] = wpi::math::SwerveModulePosition{wpi::units::meter_t{positionMeters}, angle};
    }

    // Update alerts
    driveDisconnectedAlert_.Set(!inputs_.driveConnected);
    turnDisconnectedAlert_.Set(!inputs_.turnConnected);
}

void Module::RunSetpoint(wpi::math::SwerveModuleVelocity& state) {
    // Optimize velocity setpoint
    state = state.Optimize(GetAngle());
    state = state.CosineScale(inputs_.turnPosition);

    // Apply setpoints
    io_->SetDriveVelocity(state.velocity.value() / wheelRadiusMeters);
    io_->SetTurnPosition(state.angle);
}

void Module::RunCharacterization(double output) {
    io_->SetDriveOpenLoop(output);
    io_->SetTurnPosition(wpi::math::Rotation2d{});
}

void Module::Stop() {
    io_->SetDriveOpenLoop(0.0);
    io_->SetTurnOpenLoop(0.0);
}

wpi::math::Rotation2d Module::GetAngle() const {
    return inputs_.turnPosition;
}

double Module::GetPositionMeters() const {
    return inputs_.drivePositionRad * wheelRadiusMeters;
}

double Module::GetVelocityMetersPerSec() const {
    return inputs_.driveVelocityRadPerSec * wheelRadiusMeters;
}

wpi::math::SwerveModulePosition Module::GetPosition() const {
    return {wpi::units::meter_t{GetPositionMeters()}, GetAngle()};
}

wpi::math::SwerveModuleVelocity Module::GetState() const {
    return {wpi::units::meters_per_second_t{GetVelocityMetersPerSec()}, GetAngle()};
}

const std::vector<wpi::math::SwerveModulePosition>& Module::GetOdometryPositions() const {
    return odometryPositions_;
}

const std::vector<double>& Module::GetOdometryTimestamps() const {
    return inputs_.odometryTimestamps;
}

double Module::GetWheelRadiusCharacterizationPosition() const {
    return inputs_.drivePositionRad;
}

double Module::GetFFCharacterizationVelocity() const {
    return inputs_.driveVelocityRadPerSec;
}
