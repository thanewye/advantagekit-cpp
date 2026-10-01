// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/DCMotorSim.h>

#include "subsystems/drive/ModuleIO.h"

/** Physics sim implementation of module IO. */
class ModuleIOSim : public ModuleIO {
public:
    ModuleIOSim();

    void UpdateInputs(ModuleIOInputs& inputs) override;
    void SetDriveOpenLoop(double output) override;
    void SetTurnOpenLoop(double output) override;
    void SetDriveVelocity(double velocityRadPerSec) override;
    void SetTurnPosition(const frc::Rotation2d& rotation) override;

private:
    frc::sim::DCMotorSim driveSim_;
    frc::sim::DCMotorSim turnSim_;

    bool driveClosedLoop_ = false;
    bool turnClosedLoop_ = false;
    frc::PIDController driveController_;
    frc::PIDController turnController_;
    double driveFFVolts_ = 0.0;
    double driveAppliedVolts_ = 0.0;
    double turnAppliedVolts_ = 0.0;
};
