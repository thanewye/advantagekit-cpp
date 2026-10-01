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
    feederConfig.encoder
        .PositionConversionFactor(2.0 * std::numbers::pi / feederMotorReduction) // Rotor Rotations -> Roller Radians
        .VelocityConversionFactor((2.0 * std::numbers::pi) / 60.0 / feederMotorReduction)
        .UvwMeasurementPeriod(10)
        .UvwAverageDepth(2);
    TryUntilOk(feeder_, 5, [&] {
        return feeder_.Configure(feederConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    });

    rev::spark::SparkMaxConfig intakeLauncherConfig;
    intakeLauncherConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(intakeLauncherCurrentLimit)
        .Inverted(true)
        .VoltageCompensation(12.0);
    intakeLauncherConfig.encoder
        .PositionConversionFactor(2.0 * std::numbers::pi / intakeLauncherMotorReduction) // Rotor Rotations -> Roller Radians
        .VelocityConversionFactor((2.0 * std::numbers::pi) / 60.0 / intakeLauncherMotorReduction)
        .UvwMeasurementPeriod(10)
        .UvwAverageDepth(2);
    TryUntilOk(intakeLauncher_, 5, [&] {
        return intakeLauncher_.Configure(intakeLauncherConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    });
}

void SuperstructureIOSpark::UpdateInputs(SuperstructureIOInputs& inputs) {
    IfOk(feeder_, [&] { return feederEncoder_.GetPosition(); }, [&](double value) { inputs.feederPositionRad = value; });
    IfOk(feeder_, [&] { return feederEncoder_.GetVelocity(); }, [&](double value) { inputs.feederVelocityRadPerSec = value; });
    const std::array<std::function<double()>, 2> feederVoltageSuppliers{[&] { return feeder_.GetAppliedOutput(); },
                                                                        [&] { return feeder_.GetBusVoltage(); }};
    IfOk(feeder_, feederVoltageSuppliers, [&](const std::vector<double>& values) { inputs.feederAppliedVolts = values[0] * values[1]; });
    IfOk(feeder_, [&] { return feeder_.GetOutputCurrent(); }, [&](double value) { inputs.feederCurrentAmps = value; });

    IfOk(intakeLauncher_, [&] { return intakeLauncherEncoder_.GetPosition(); }, [&](double value) { inputs.intakeLauncherPositionRad = value; });
    IfOk(intakeLauncher_, [&] { return intakeLauncherEncoder_.GetVelocity(); },
         [&](double value) { inputs.intakeLauncherVelocityRadPerSec = value; });
    const std::array<std::function<double()>, 2> intakeLauncherVoltageSuppliers{[&] { return intakeLauncher_.GetAppliedOutput(); },
                                                                                [&] { return intakeLauncher_.GetBusVoltage(); }};
    IfOk(intakeLauncher_, intakeLauncherVoltageSuppliers,
         [&](const std::vector<double>& values) { inputs.intakeLauncherAppliedVolts = values[0] * values[1]; });
    IfOk(intakeLauncher_, [&] { return intakeLauncher_.GetOutputCurrent(); }, [&](double value) { inputs.intakeLauncherCurrentAmps = value; });
}

void SuperstructureIOSpark::SetFeederVoltage(double volts) {
    feeder_.SetVoltage(units::volt_t{volts});
}

void SuperstructureIOSpark::SetIntakeLauncherVoltage(double volts) {
    intakeLauncher_.SetVoltage(units::volt_t{volts});
}
