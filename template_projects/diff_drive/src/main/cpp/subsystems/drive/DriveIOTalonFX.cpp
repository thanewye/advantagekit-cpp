// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/DriveIOTalonFX.h"

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <wpi/units/frequency.hpp>

#include "util/PhoenixUtil.h"

using namespace DriveConstants;
using namespace PhoenixUtil;

DriveIOTalonFX::DriveIOTalonFX() {
    ctre::phoenix6::configs::TalonFXConfiguration config;
    config.CurrentLimits.SupplyCurrentLimit = wpi::units::ampere_t{static_cast<double>(currentLimit)};
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    config.Feedback.SensorToMechanismRatio = motorReduction;
    config.Slot0.kP = realKp;
    config.Slot0.kD = realKd;

    config.MotorOutput.Inverted =
        leftInverted ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    TryUntilOk(5, [&] { return leftLeader_.GetConfigurator().Apply(config, 0.25_s); });
    TryUntilOk(5, [&] { return leftFollower_.GetConfigurator().Apply(config, 0.25_s); });

    config.MotorOutput.Inverted =
        rightInverted ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    TryUntilOk(5, [&] { return rightLeader_.GetConfigurator().Apply(config, 0.25_s); });
    TryUntilOk(5, [&] { return rightFollower_.GetConfigurator().Apply(config, 0.25_s); });

    leftFollower_.SetControl(ctre::phoenix6::controls::Follower{leftLeader_.GetDeviceID(), ctre::phoenix6::signals::MotorAlignmentValue::Aligned});
    rightFollower_.SetControl(ctre::phoenix6::controls::Follower{rightLeader_.GetDeviceID(), ctre::phoenix6::signals::MotorAlignmentValue::Aligned});

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, leftPosition_, leftVelocity_, leftAppliedVolts_, leftLeaderCurrent_, leftFollowerCurrent_,
                                                               rightPosition_, rightVelocity_, rightAppliedVolts_, rightLeaderCurrent_, rightFollowerCurrent_);
    leftLeader_.OptimizeBusUtilization();
    leftFollower_.OptimizeBusUtilization();
    rightLeader_.OptimizeBusUtilization();
    rightFollower_.OptimizeBusUtilization();
}

void DriveIOTalonFX::UpdateInputs(DriveIOInputs& inputs) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(leftPosition_, leftVelocity_, leftAppliedVolts_, leftLeaderCurrent_, leftFollowerCurrent_, rightPosition_,
                                                 rightVelocity_, rightAppliedVolts_, rightLeaderCurrent_, rightFollowerCurrent_);

    inputs.leftPositionRad = wpi::units::radian_t{leftPosition_.GetValue()}.value();
    inputs.leftVelocityRadPerSec = wpi::units::radians_per_second_t{leftVelocity_.GetValue()}.value();
    inputs.leftAppliedVolts = leftAppliedVolts_.GetValue().value();
    inputs.leftCurrentAmps = {leftLeaderCurrent_.GetValue().value(), leftFollowerCurrent_.GetValue().value()};

    inputs.rightPositionRad = wpi::units::radian_t{rightPosition_.GetValue()}.value();
    inputs.rightVelocityRadPerSec = wpi::units::radians_per_second_t{rightVelocity_.GetValue()}.value();
    inputs.rightAppliedVolts = rightAppliedVolts_.GetValue().value();
    inputs.rightCurrentAmps = {rightLeaderCurrent_.GetValue().value(), rightFollowerCurrent_.GetValue().value()};
}

void DriveIOTalonFX::SetVoltage(double leftVolts, double rightVolts) {
    leftLeader_.SetControl(voltageRequest_.WithOutput(wpi::units::volt_t{leftVolts}));
    rightLeader_.SetControl(voltageRequest_.WithOutput(wpi::units::volt_t{rightVolts}));
}

void DriveIOTalonFX::SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    leftLeader_.SetControl(velocityRequest_.WithVelocity(wpi::units::radians_per_second_t{leftRadPerSec}).WithFeedForward(wpi::units::volt_t{leftFFVolts}));
    rightLeader_.SetControl(velocityRequest_.WithVelocity(wpi::units::radians_per_second_t{rightRadPerSec}).WithFeedForward(wpi::units::volt_t{rightFFVolts}));
}
