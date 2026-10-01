// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"

class DriveIOSim : public DriveIO {
public:
    void UpdateInputs(DriveIOInputs& inputs) override;
    void SetVoltage(double leftVolts, double rightVolts) override;
    void SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) override;

private:
    frc::sim::DifferentialDrivetrainSim sim_ = frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
        frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,
        frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch);

    double leftAppliedVolts_ = 0.0;
    double rightAppliedVolts_ = 0.0;
    bool closedLoop_ = false;
    frc::PIDController leftPID_{DriveConstants::simKp, 0.0, DriveConstants::simKd};
    frc::PIDController rightPID_{DriveConstants::simKp, 0.0, DriveConstants::simKd};
    double leftFFVolts_ = 0.0;
    double rightFFVolts_ = 0.0;
};
