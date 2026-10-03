// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <wpi/math/controller/PIDController.hpp>
#include <wpi/simulation/DifferentialDrivetrainSim.hpp>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"

class DriveIOSim : public DriveIO {
public:
    void UpdateInputs(DriveIOInputs& inputs) override;
    void SetVoltage(double leftVolts, double rightVolts) override;
    void SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) override;

private:
    wpi::sim::DifferentialDrivetrainSim sim_ = wpi::sim::DifferentialDrivetrainSim::CreateKitbotSim(
        wpi::sim::DifferentialDrivetrainSim::KitbotMotor::DUAL_CIM_PER_SIDE, wpi::sim::DifferentialDrivetrainSim::KitbotGearing::RATIO_10P71,
        wpi::sim::DifferentialDrivetrainSim::KitbotWheelSize::SIX_INCH);

    double leftAppliedVolts_ = 0.0;
    double rightAppliedVolts_ = 0.0;
    bool closedLoop_ = false;
    wpi::math::PIDController leftPID_{DriveConstants::simKp, 0.0, DriveConstants::simKd};
    wpi::math::PIDController rightPID_{DriveConstants::simKp, 0.0, DriveConstants::simKd};
    double leftFFVolts_ = 0.0;
    double rightFFVolts_ = 0.0;
};
