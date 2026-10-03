// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/simulation/DCMotorSim.hpp>

#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructureIO.h"

class SuperstructureIOSim : public SuperstructureIO {
public:
    void UpdateInputs(SuperstructureIOInputs& inputs) override;
    void SetFeederVoltage(double volts) override;
    void SetIntakeLauncherVoltage(double volts) override;

private:
    wpi::sim::DCMotorSim feederSim_{
        wpi::math::Models::SingleJointedArmFromPhysicalConstants(wpi::math::DCMotor::CIM(1), 0.004_kg_sq_m, SuperstructureConstants::feederMotorReduction),
        wpi::math::DCMotor::CIM(1)};
    wpi::sim::DCMotorSim intakeLauncherSim_{wpi::math::Models::SingleJointedArmFromPhysicalConstants(wpi::math::DCMotor::CIM(1), 0.004_kg_sq_m,
                                                                                                     SuperstructureConstants::intakeLauncherMotorReduction),
                                            wpi::math::DCMotor::CIM(1)};

    double feederAppliedVolts_ = 0.0;
    double intakeLauncherAppliedVolts_ = 0.0;
};
