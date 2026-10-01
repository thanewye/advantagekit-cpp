// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/superstructure/SuperstructureIOTalonSRX.h"

#include "util/PhoenixUtil.h"

using namespace ctre::phoenix::motorcontrol;
using namespace SuperstructureConstants;
using namespace PhoenixUtil;

SuperstructureIOTalonSRX::SuperstructureIOTalonSRX() {
    can::TalonSRXConfiguration feederConfig;
    feederConfig.peakCurrentLimit = feederCurrentLimit;
    feederConfig.continuousCurrentLimit = feederCurrentLimit - 15;
    feederConfig.peakCurrentDuration = 250;
    feederConfig.voltageCompSaturation = 12.0;
    TryUntilOkV5(5, [&] { return feeder_.ConfigAllSettings(feederConfig); });

    can::TalonSRXConfiguration intakeLauncherConfig;
    intakeLauncherConfig.peakCurrentLimit = intakeLauncherCurrentLimit;
    intakeLauncherConfig.continuousCurrentLimit = intakeLauncherCurrentLimit - 15;
    intakeLauncherConfig.peakCurrentDuration = 250;
    intakeLauncherConfig.voltageCompSaturation = 12.0;
    TryUntilOkV5(5, [&] { return intakeLauncher_.ConfigAllSettings(intakeLauncherConfig); });
    intakeLauncher_.SetInverted(true);
}

void SuperstructureIOTalonSRX::UpdateInputs(SuperstructureIOInputs& inputs) {
    inputs.feederAppliedVolts = feeder_.GetMotorOutputVoltage();
    inputs.feederCurrentAmps = feeder_.GetStatorCurrent();
    inputs.intakeLauncherAppliedVolts = intakeLauncher_.GetMotorOutputVoltage();
    inputs.intakeLauncherCurrentAmps = intakeLauncher_.GetStatorCurrent();
}

void SuperstructureIOTalonSRX::SetFeederVoltage(double volts) {
    feeder_.Set(TalonSRXControlMode::PercentOutput, volts / 12.0);
}

void SuperstructureIOTalonSRX::SetIntakeLauncherVoltage(double volts) {
    intakeLauncher_.Set(TalonSRXControlMode::PercentOutput, volts / 12.0);
}
