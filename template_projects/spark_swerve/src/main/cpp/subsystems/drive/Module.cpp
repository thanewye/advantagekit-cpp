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
    , driveDisconnectedAlert_("Disconnected drive motor on module " + std::to_string(index) + ".", frc::Alert::AlertType::kError)
    , turnDisconnectedAlert_("Disconnected turn motor on module " + std::to_string(index) + ".", frc::Alert::AlertType::kError) {}

void Module::Periodic() {
    io_->UpdateInputs(inputs_);
    akit::Logger::ProcessInputs("Drive/Module" + std::to_string(index_), inputs_);

    // Calculate positions for odometry
    size_t sampleCount = inputs_.odometryTimestamps.size(); // All signals are sampled together
    odometryPositions_.resize(sampleCount);
    for (size_t i = 0; i < sampleCount; i++) {
        double positionMeters = inputs_.odometryDrivePositionsRad[i] * wheelRadiusMeters;
        frc::Rotation2d angle = inputs_.odometryTurnPositions[i];
        odometryPositions_[i] = frc::SwerveModulePosition{units::meter_t{positionMeters}, angle};
    }

    // Update alerts
    driveDisconnectedAlert_.Set(!inputs_.driveConnected);
    turnDisconnectedAlert_.Set(!inputs_.turnConnected);
}

void Module::RunSetpoint(frc::SwerveModuleState& state) {
    // Optimize velocity setpoint
    state.Optimize(GetAngle());
    state.CosineScale(inputs_.turnPosition);

    // Apply setpoints
    io_->SetDriveVelocity(state.speed.value() / wheelRadiusMeters);
    io_->SetTurnPosition(state.angle);
}

void Module::RunCharacterization(double output) {
    io_->SetDriveOpenLoop(output);
    io_->SetTurnPosition(frc::Rotation2d{});
}

void Module::Stop() {
    io_->SetDriveOpenLoop(0.0);
    io_->SetTurnOpenLoop(0.0);
}

frc::Rotation2d Module::GetAngle() const {
    return inputs_.turnPosition;
}

double Module::GetPositionMeters() const {
    return inputs_.drivePositionRad * wheelRadiusMeters;
}

double Module::GetVelocityMetersPerSec() const {
    return inputs_.driveVelocityRadPerSec * wheelRadiusMeters;
}

frc::SwerveModulePosition Module::GetPosition() const {
    return {units::meter_t{GetPositionMeters()}, GetAngle()};
}

frc::SwerveModuleState Module::GetState() const {
    return {units::meters_per_second_t{GetVelocityMetersPerSec()}, GetAngle()};
}

const std::vector<frc::SwerveModulePosition>& Module::GetOdometryPositions() const {
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
