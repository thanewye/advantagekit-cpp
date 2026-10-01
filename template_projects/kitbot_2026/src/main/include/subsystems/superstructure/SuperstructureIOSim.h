// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructureIO.h"

class SuperstructureIOSim : public SuperstructureIO {
public:
    void UpdateInputs(SuperstructureIOInputs& inputs) override;
    void SetFeederVoltage(double volts) override;
    void SetIntakeLauncherVoltage(double volts) override;

private:
    frc::sim::DCMotorSim feederSim_{
        frc::LinearSystemId::DCMotorSystem(frc::DCMotor::CIM(1), 0.004_kg_sq_m, SuperstructureConstants::feederMotorReduction),
        frc::DCMotor::CIM(1)};
    frc::sim::DCMotorSim intakeLauncherSim_{
        frc::LinearSystemId::DCMotorSystem(frc::DCMotor::CIM(1), 0.004_kg_sq_m, SuperstructureConstants::intakeLauncherMotorReduction),
        frc::DCMotor::CIM(1)};

    double feederAppliedVolts_ = 0.0;
    double intakeLauncherAppliedVolts_ = 0.0;
};
