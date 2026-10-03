// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/DriveIOSpark.h"

#include <array>
#include <functional>
#include <numbers>

#include <rev/config/SparkMaxConfig.h>

#include "util/SparkUtil.h"

using namespace DriveConstants;
using namespace SparkUtil;

DriveIOSpark::DriveIOSpark() {
    // Create config
    rev::spark::SparkMaxConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake).SmartCurrentLimit(currentLimit).VoltageCompensation(12.0);
    config.closedLoop.Pid(realKp * (2 * std::numbers::pi / 60.0 / motorReduction), 0.0, realKd * (2 * std::numbers::pi / 60.0 / motorReduction));

    // Apply config to leaders
    config.Inverted(leftInverted);
    TryUntilOk(leftLeader_, 5, [&] { return leftLeader_.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });
    config.Inverted(rightInverted);
    TryUntilOk(rightLeader_, 5, [&] { return rightLeader_.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });

    // Apply config to followers
    config.Inverted(leftInverted).Follow(leftLeader_);
    TryUntilOk(leftFollower_, 5, [&] { return leftFollower_.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });
    config.Inverted(rightInverted).Follow(rightLeader_);
    TryUntilOk(rightFollower_, 5, [&] { return rightFollower_.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });
}

void DriveIOSpark::UpdateInputs(DriveIOInputs& inputs) {
    IfOk([&] { return leftEncoder_.GetPosition(); }, [&](double value) { inputs.leftPositionRad = value * (2 * std::numbers::pi / motorReduction); });
    IfOk([&] { return leftEncoder_.GetVelocity(); },
         [&](double value) { inputs.leftVelocityRadPerSec = value * (2 * std::numbers::pi / 60.0 / motorReduction); });
    const std::array<std::function<rev::util::Signal<double>()>, 2> leftVoltageSuppliers{[&] { return leftLeader_.GetAppliedOutput(); },
                                                                                         [&] { return leftLeader_.GetBusVoltage(); }};
    IfOk(leftVoltageSuppliers, [&](const std::vector<double>& values) { inputs.leftAppliedVolts = values[0] * values[1]; });
    const std::array<std::function<rev::util::Signal<double>()>, 2> leftCurrentSuppliers{[&] { return leftLeader_.GetOutputCurrent(); },
                                                                                         [&] { return leftFollower_.GetOutputCurrent(); }};
    IfOk(leftCurrentSuppliers, [&](const std::vector<double>& values) { inputs.leftCurrentAmps = values; });

    IfOk([&] { return rightEncoder_.GetPosition(); }, [&](double value) { inputs.rightPositionRad = value * (2 * std::numbers::pi / motorReduction); });
    IfOk([&] { return rightEncoder_.GetVelocity(); },
         [&](double value) { inputs.rightVelocityRadPerSec = value * (2 * std::numbers::pi / 60.0 / motorReduction); });
    const std::array<std::function<rev::util::Signal<double>()>, 2> rightVoltageSuppliers{[&] { return rightLeader_.GetAppliedOutput(); },
                                                                                          [&] { return rightLeader_.GetBusVoltage(); }};
    IfOk(rightVoltageSuppliers, [&](const std::vector<double>& values) { inputs.rightAppliedVolts = values[0] * values[1]; });
    const std::array<std::function<rev::util::Signal<double>()>, 2> rightCurrentSuppliers{[&] { return rightLeader_.GetOutputCurrent(); },
                                                                                          [&] { return rightLeader_.GetOutputCurrent(); }};
    IfOk(rightCurrentSuppliers, [&](const std::vector<double>& values) { inputs.rightCurrentAmps = values; });
}

void DriveIOSpark::SetVoltage(double leftVolts, double rightVolts) {
    leftLeader_.SetVoltage(wpi::units::volt_t{leftVolts});
    rightLeader_.SetVoltage(wpi::units::volt_t{rightVolts});
}

void DriveIOSpark::SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    leftController_.SetSetpoint(leftRadPerSec / (2 * std::numbers::pi / 60.0 / motorReduction), rev::spark::SparkLowLevel::ControlType::kVelocity,
                                rev::spark::ClosedLoopSlot::kSlot0, leftFFVolts);
    rightController_.SetSetpoint(rightRadPerSec / (2 * std::numbers::pi / 60.0 / motorReduction), rev::spark::SparkLowLevel::ControlType::kVelocity,
                                 rev::spark::ClosedLoopSlot::kSlot0, rightFFVolts);
}
