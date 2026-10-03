// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <rev/SparkMax.h>

#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/SuperstructureIO.h"

/**
 * This superstructure implementation is for Spark devices. It defaults to brushless control, but
 * can be easily adapted for a brushed motor. One or more Spark Flexes can be used by swapping
 * relevant instances of "SparkMax" with "SparkFlex".
 */
class SuperstructureIOSpark : public SuperstructureIO {
public:
    SuperstructureIOSpark();

    void UpdateInputs(SuperstructureIOInputs& inputs) override;
    void SetFeederVoltage(double volts) override;
    void SetIntakeLauncherVoltage(double volts) override;

private:
    rev::spark::SparkMax feeder_{wpi::CANPort::CAN_S0, SuperstructureConstants::feederCanId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax intakeLauncher_{wpi::CANPort::CAN_S0, SuperstructureConstants::intakeLauncherCanId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder& feederEncoder_ = feeder_.GetEncoder();
    rev::spark::SparkRelativeEncoder& intakeLauncherEncoder_ = intakeLauncher_.GetEncoder();
};
