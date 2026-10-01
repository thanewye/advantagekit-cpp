// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <rev/SparkMax.h>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"

/**
 * This drive implementation is for Spark devices. It defaults to brushless control, but can be
 * easily adapted for brushed motors and external encoders. Spark Flexes can be used by swapping all
 * instances of "SparkMax" with "SparkFlex".
 */
class DriveIOSpark : public DriveIO {
public:
    DriveIOSpark();

    void UpdateInputs(DriveIOInputs& inputs) override;
    void SetVoltage(double leftVolts, double rightVolts) override;
    void SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) override;

private:
    rev::spark::SparkMax leftLeader_{DriveConstants::leftLeaderCanId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax rightLeader_{DriveConstants::rightLeaderCanId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax leftFollower_{DriveConstants::leftFollowerCanId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax rightFollower_{DriveConstants::rightFollowerCanId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder& leftEncoder_ = leftLeader_.GetEncoder();
    rev::spark::SparkRelativeEncoder& rightEncoder_ = rightLeader_.GetEncoder();
    rev::spark::SparkClosedLoopController& leftController_ = leftLeader_.GetClosedLoopController();
    rev::spark::SparkClosedLoopController& rightController_ = rightLeader_.GetClosedLoopController();
};
