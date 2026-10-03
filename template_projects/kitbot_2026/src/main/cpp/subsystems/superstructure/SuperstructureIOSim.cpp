// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/superstructure/SuperstructureIOSim.h"

#include <algorithm>

void SuperstructureIOSim::UpdateInputs(SuperstructureIOInputs& inputs) {
    feederSim_.SetInputVoltage(wpi::units::volt_t{feederAppliedVolts_});
    feederSim_.Update(20_ms);

    intakeLauncherSim_.SetInputVoltage(wpi::units::volt_t{intakeLauncherAppliedVolts_});
    intakeLauncherSim_.Update(20_ms);

    inputs.feederPositionRad = feederSim_.GetAngularPosition().value();
    inputs.feederVelocityRadPerSec = feederSim_.GetAngularVelocity().value();
    inputs.feederAppliedVolts = feederAppliedVolts_;
    inputs.feederCurrentAmps = feederSim_.GetCurrentDraw().value();

    inputs.intakeLauncherPositionRad = intakeLauncherSim_.GetAngularPosition().value();
    inputs.intakeLauncherVelocityRadPerSec = intakeLauncherSim_.GetAngularVelocity().value();
    inputs.intakeLauncherAppliedVolts = intakeLauncherAppliedVolts_;
    inputs.intakeLauncherCurrentAmps = intakeLauncherSim_.GetCurrentDraw().value();
}

void SuperstructureIOSim::SetFeederVoltage(double volts) {
    feederAppliedVolts_ = std::clamp(volts, -12.0, 12.0);
}

void SuperstructureIOSim::SetIntakeLauncherVoltage(double volts) {
    intakeLauncherAppliedVolts_ = std::clamp(volts, -12.0, 12.0);
}
