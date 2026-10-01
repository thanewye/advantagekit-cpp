// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructureIO.h"

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
class SuperstructureIOTalonFX : public SuperstructureIO {
public:
    SuperstructureIOTalonFX();

    void UpdateInputs(SuperstructureIOInputs& inputs) override;
    void SetFeederVoltage(double volts) override;
    void SetIntakeLauncherVoltage(double volts) override;

private:
    ctre::phoenix6::hardware::TalonFX feeder_{SuperstructureConstants::feederCanId};
    ctre::phoenix6::StatusSignal<units::turn_t> feederPositionRot_ = feeder_.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> feederVelocityRotPerSec_ = feeder_.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> feederAppliedVolts_ = feeder_.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> feederCurrentAmps_ = feeder_.GetSupplyCurrent();

    ctre::phoenix6::hardware::TalonFX intakeLauncher_{SuperstructureConstants::intakeLauncherCanId};
    ctre::phoenix6::StatusSignal<units::turn_t> intakeLauncherPositionRot_ = intakeLauncher_.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> intakeLauncherVelocityRotPerSec_ = intakeLauncher_.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> intakeLauncherAppliedVolts_ = intakeLauncher_.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> intakeLauncherCurrentAmps_ = intakeLauncher_.GetSupplyCurrent();

    ctre::phoenix6::controls::VoltageOut voltageRequest_{0_V};
};
