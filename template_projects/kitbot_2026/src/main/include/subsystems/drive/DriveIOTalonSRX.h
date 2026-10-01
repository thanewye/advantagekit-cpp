// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"

/** This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) with encoders. */
class DriveIOTalonSRX : public DriveIO {
public:
    DriveIOTalonSRX();

    void UpdateInputs(DriveIOInputs& inputs) override;
    void SetVoltage(double leftVolts, double rightVolts) override;
    void SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) override;

private:
    static constexpr double ticksPerRevolution = 1440;

    ctre::phoenix::motorcontrol::can::TalonSRX leftLeader_{DriveConstants::leftLeaderCanId};
    ctre::phoenix::motorcontrol::can::TalonSRX leftFollower_{DriveConstants::leftFollowerCanId};
    ctre::phoenix::motorcontrol::can::TalonSRX rightLeader_{DriveConstants::rightLeaderCanId};
    ctre::phoenix::motorcontrol::can::TalonSRX rightFollower_{DriveConstants::rightFollowerCanId};
};
