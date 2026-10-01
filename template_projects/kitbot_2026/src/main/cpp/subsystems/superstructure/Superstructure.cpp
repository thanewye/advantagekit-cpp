// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/superstructure/Superstructure.h"

#include <akit/Logger.h>
#include <units/time.h>

#include "subsystems/superstructure/SuperstructureConstants.h"

using namespace SuperstructureConstants;

Superstructure::Superstructure(std::unique_ptr<SuperstructureIO> io)
    : io_(std::move(io)) {}

void Superstructure::Periodic() {
    io_->UpdateInputs(inputs_);
    akit::Logger::ProcessInputs("Superstructure", inputs_);
}

frc2::CommandPtr Superstructure::Intake() {
    return RunEnd(
        [this] {
            io_->SetFeederVoltage(intakingFeederVoltage);
            io_->SetIntakeLauncherVoltage(intakingFeederVoltage);
        },
        [this] {
            io_->SetFeederVoltage(0.0);
            io_->SetIntakeLauncherVoltage(0.0);
        });
}

frc2::CommandPtr Superstructure::Eject() {
    return RunEnd(
        [this] {
            io_->SetFeederVoltage(-intakingFeederVoltage);
            io_->SetIntakeLauncherVoltage(-intakingFeederVoltage);
        },
        [this] {
            io_->SetFeederVoltage(0.0);
            io_->SetIntakeLauncherVoltage(0.0);
        });
}

frc2::CommandPtr Superstructure::Launch() {
    return Run([this] {
               io_->SetFeederVoltage(spinUpFeederVoltage);
               io_->SetIntakeLauncherVoltage(launchingLauncherVoltage);
           })
        .WithTimeout(units::second_t{spinUpSeconds})
        .AndThen(Run([this] {
            io_->SetFeederVoltage(launchingFeederVoltage);
            io_->SetIntakeLauncherVoltage(launchingLauncherVoltage);
        }))
        .FinallyDo([this] {
            io_->SetFeederVoltage(0.0);
            io_->SetIntakeLauncherVoltage(0.0);
        });
}
