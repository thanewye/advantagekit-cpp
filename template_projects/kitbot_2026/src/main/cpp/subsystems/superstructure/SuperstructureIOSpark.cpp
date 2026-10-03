// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/superstructure/SuperstructureIOSpark.h"

#include <array>
#include <functional>
#include <numbers>

#include <rev/config/SparkMaxConfig.h>

#include "util/SparkUtil.h"

using namespace SuperstructureConstants;
using namespace SparkUtil;

SuperstructureIOSpark::SuperstructureIOSpark() {
    rev::spark::SparkMaxConfig feederConfig;
    feederConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake).SmartCurrentLimit(feederCurrentLimit).VoltageCompensation(12.0);
    TryUntilOk(feeder_, 5, [&] { return feeder_.Configure(feederConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });

    rev::spark::SparkMaxConfig intakeLauncherConfig;
    intakeLauncherConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(intakeLauncherCurrentLimit)
        .Inverted(true)
        .VoltageCompensation(12.0);
    TryUntilOk(intakeLauncher_, 5,
               [&] { return intakeLauncher_.Configure(intakeLauncherConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });
}

void SuperstructureIOSpark::UpdateInputs(SuperstructureIOInputs& inputs) {
    IfOk([&] { return feederEncoder_.GetPosition(); }, [&](double value) { inputs.feederPositionRad = value * (2 * std::numbers::pi / feederMotorReduction); });
    IfOk([&] { return feederEncoder_.GetVelocity(); },
         [&](double value) { inputs.feederVelocityRadPerSec = value * (2 * std::numbers::pi / 60.0 / feederMotorReduction); });
    const std::array<std::function<rev::util::Signal<double>()>, 2> feederVoltageSuppliers{[&] { return feeder_.GetAppliedOutput(); },
                                                                                           [&] { return feeder_.GetBusVoltage(); }};
    IfOk(feederVoltageSuppliers, [&](const std::vector<double>& values) { inputs.feederAppliedVolts = values[0] * values[1]; });
    IfOk([&] { return feeder_.GetOutputCurrent(); }, [&](double value) { inputs.feederCurrentAmps = value; });

    IfOk([&] { return intakeLauncherEncoder_.GetPosition(); },
         [&](double value) { inputs.intakeLauncherPositionRad = value * (2 * std::numbers::pi / intakeLauncherMotorReduction); });
    IfOk([&] { return intakeLauncherEncoder_.GetVelocity(); },
         [&](double value) { inputs.intakeLauncherVelocityRadPerSec = value * (2 * std::numbers::pi / 60.0 / intakeLauncherMotorReduction); });
    const std::array<std::function<rev::util::Signal<double>()>, 2> intakeLauncherVoltageSuppliers{[&] { return intakeLauncher_.GetAppliedOutput(); },
                                                                                                   [&] { return intakeLauncher_.GetBusVoltage(); }};
    IfOk(intakeLauncherVoltageSuppliers, [&](const std::vector<double>& values) { inputs.intakeLauncherAppliedVolts = values[0] * values[1]; });
    IfOk([&] { return intakeLauncher_.GetOutputCurrent(); }, [&](double value) { inputs.intakeLauncherCurrentAmps = value; });
}

void SuperstructureIOSpark::SetFeederVoltage(double volts) {
    feeder_.SetVoltage(wpi::units::volt_t{volts});
}

void SuperstructureIOSpark::SetIntakeLauncherVoltage(double volts) {
    intakeLauncher_.SetVoltage(wpi::units::volt_t{volts});
}
