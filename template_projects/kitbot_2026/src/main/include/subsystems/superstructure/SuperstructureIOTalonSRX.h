// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructureIO.h"

/** This superstructure implementation is for Talon SRXs driving brushed motors. */
class SuperstructureIOTalonSRX : public SuperstructureIO {
public:
    SuperstructureIOTalonSRX();

    void UpdateInputs(SuperstructureIOInputs& inputs) override;
    void SetFeederVoltage(double volts) override;
    void SetIntakeLauncherVoltage(double volts) override;

private:
    ctre::phoenix::motorcontrol::can::TalonSRX feeder_{SuperstructureConstants::feederCanId};
    ctre::phoenix::motorcontrol::can::TalonSRX intakeLauncher_{SuperstructureConstants::intakeLauncherCanId};
};
