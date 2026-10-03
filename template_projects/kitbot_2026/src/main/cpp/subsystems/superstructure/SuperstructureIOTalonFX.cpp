// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/superstructure/SuperstructureIOTalonFX.h"

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <wpi/units/frequency.hpp>

#include "util/PhoenixUtil.h"

using namespace SuperstructureConstants;
using namespace PhoenixUtil;

SuperstructureIOTalonFX::SuperstructureIOTalonFX() {
    ctre::phoenix6::configs::TalonFXConfiguration feederConfig;
    feederConfig.CurrentLimits.SupplyCurrentLimit = wpi::units::ampere_t{static_cast<double>(feederCurrentLimit)};
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    TryUntilOk(5, [&] { return feeder_.GetConfigurator().Apply(feederConfig, 0.25_s); });

    ctre::phoenix6::configs::TalonFXConfiguration intakeLauncherConfig;
    intakeLauncherConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    intakeLauncherConfig.CurrentLimits.SupplyCurrentLimit = wpi::units::ampere_t{static_cast<double>(intakeLauncherCurrentLimit)};
    intakeLauncherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeLauncherConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    TryUntilOk(5, [&] { return intakeLauncher_.GetConfigurator().Apply(intakeLauncherConfig, 0.25_s); });

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, feederPositionRot_, feederVelocityRotPerSec_, feederAppliedVolts_, feederCurrentAmps_,
                                                               intakeLauncherPositionRot_, intakeLauncherVelocityRotPerSec_, intakeLauncherAppliedVolts_,
                                                               intakeLauncherCurrentAmps_);
    ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(feeder_, intakeLauncher_);
}

void SuperstructureIOTalonFX::UpdateInputs(SuperstructureIOInputs& inputs) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(feederPositionRot_, feederVelocityRotPerSec_, feederAppliedVolts_, feederCurrentAmps_,
                                                 intakeLauncherPositionRot_, intakeLauncherVelocityRotPerSec_, intakeLauncherAppliedVolts_,
                                                 intakeLauncherCurrentAmps_);

    inputs.feederPositionRad = wpi::units::radian_t{feederPositionRot_.GetValue()}.value();
    inputs.feederVelocityRadPerSec = wpi::units::radians_per_second_t{feederVelocityRotPerSec_.GetValue()}.value();
    inputs.feederAppliedVolts = feederAppliedVolts_.GetValue().value();
    inputs.feederCurrentAmps = feederCurrentAmps_.GetValue().value();
    inputs.intakeLauncherPositionRad = wpi::units::radian_t{intakeLauncherPositionRot_.GetValue()}.value();
    inputs.intakeLauncherVelocityRadPerSec = wpi::units::radians_per_second_t{intakeLauncherVelocityRotPerSec_.GetValue()}.value();
    inputs.intakeLauncherAppliedVolts = intakeLauncherAppliedVolts_.GetValue().value();
    inputs.intakeLauncherCurrentAmps = intakeLauncherCurrentAmps_.GetValue().value();
}

void SuperstructureIOTalonFX::SetFeederVoltage(double volts) {
    feeder_.SetControl(voltageRequest_.WithOutput(wpi::units::volt_t{volts}));
}

void SuperstructureIOTalonFX::SetIntakeLauncherVoltage(double volts) {
    intakeLauncher_.SetControl(voltageRequest_.WithOutput(wpi::units::volt_t{volts}));
}
