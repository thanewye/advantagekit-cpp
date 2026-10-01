// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/DriveIOTalonSRX.h"

#include <numbers>

#include "util/PhoenixUtil.h"

using namespace ctre::phoenix::motorcontrol;
using namespace DriveConstants;
using namespace PhoenixUtil;

DriveIOTalonSRX::DriveIOTalonSRX() {
    can::TalonSRXConfiguration config;
    config.peakCurrentLimit = currentLimit;
    config.continuousCurrentLimit = currentLimit - 15;
    config.peakCurrentDuration = 250;
    config.voltageCompSaturation = 12.0;
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;

    TryUntilOkV5(5, [&] { return leftLeader_.ConfigAllSettings(config); });
    TryUntilOkV5(5, [&] { return leftFollower_.ConfigAllSettings(config); });
    TryUntilOkV5(5, [&] { return rightLeader_.ConfigAllSettings(config); });
    TryUntilOkV5(5, [&] { return rightFollower_.ConfigAllSettings(config); });

    leftLeader_.SetInverted(leftInverted);
    rightLeader_.SetInverted(rightInverted);

    leftFollower_.Follow(leftLeader_);
    rightFollower_.Follow(rightLeader_);
}

void DriveIOTalonSRX::UpdateInputs(DriveIOInputs& inputs) {
    inputs.leftPositionRad = leftLeader_.GetSelectedSensorPosition() / ticksPerRevolution * 2 * std::numbers::pi;
    inputs.leftVelocityRadPerSec =
        leftLeader_.GetSelectedSensorVelocity() / ticksPerRevolution * 10.0 * 2 * std::numbers::pi; // Raw units are ticks per 100ms :(
    inputs.leftAppliedVolts = leftLeader_.GetMotorOutputVoltage();
    inputs.leftCurrentAmps = {leftLeader_.GetStatorCurrent(), leftFollower_.GetStatorCurrent()};

    inputs.rightPositionRad = rightLeader_.GetSelectedSensorPosition() / ticksPerRevolution * 2 * std::numbers::pi;
    inputs.rightVelocityRadPerSec =
        rightLeader_.GetSelectedSensorVelocity() / ticksPerRevolution * 10.0 * 2 * std::numbers::pi; // Raw units are ticks per 100ms :(
    inputs.rightAppliedVolts = rightLeader_.GetMotorOutputVoltage();
    inputs.rightCurrentAmps = {rightLeader_.GetStatorCurrent(), rightFollower_.GetStatorCurrent()};
}

void DriveIOTalonSRX::SetVoltage(double leftVolts, double rightVolts) {
    // OK to just divide by 12 because voltage compensation is enabled
    leftLeader_.Set(TalonSRXControlMode::PercentOutput, leftVolts / 12.0);
    rightLeader_.Set(TalonSRXControlMode::PercentOutput, rightVolts / 12.0);
}

void DriveIOTalonSRX::SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    // OK to just divide FF by 12 because voltage compensation is enabled
    leftLeader_.Set(TalonSRXControlMode::Velocity,
                    leftRadPerSec / (2 * std::numbers::pi) * ticksPerRevolution / 10.0, // Raw units are ticks per 100ms :(
                    DemandType::DemandType_ArbitraryFeedForward, leftFFVolts / 12.0);
    rightLeader_.Set(TalonSRXControlMode::Velocity,
                     rightRadPerSec / (2 * std::numbers::pi) * ticksPerRevolution / 10.0, // Raw units are ticks per 100ms :(
                     DemandType::DemandType_ArbitraryFeedForward, rightFFVolts / 12.0);
}
