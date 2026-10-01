// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/DriveIOSim.h"

#include <algorithm>

using namespace DriveConstants;

void DriveIOSim::UpdateInputs(DriveIOInputs& inputs) {
    if (closedLoop_) {
        leftAppliedVolts_ = leftFFVolts_ + leftPID_.Calculate(sim_.GetLeftVelocity().value() / wheelRadiusMeters);
        rightAppliedVolts_ = rightFFVolts_ + rightPID_.Calculate(sim_.GetRightVelocity().value() / wheelRadiusMeters);
    }

    // Update simulation state
    sim_.SetInputs(units::volt_t{std::clamp(leftAppliedVolts_, -12.0, 12.0)}, units::volt_t{std::clamp(rightAppliedVolts_, -12.0, 12.0)});
    sim_.Update(20_ms);

    inputs.leftPositionRad = sim_.GetLeftPosition().value() / wheelRadiusMeters;
    inputs.leftVelocityRadPerSec = sim_.GetLeftVelocity().value() / wheelRadiusMeters;
    inputs.leftAppliedVolts = leftAppliedVolts_;
    inputs.leftCurrentAmps = {sim_.GetLeftCurrentDraw().value()};

    inputs.rightPositionRad = sim_.GetRightPosition().value() / wheelRadiusMeters;
    inputs.rightVelocityRadPerSec = sim_.GetRightVelocity().value() / wheelRadiusMeters;
    inputs.rightAppliedVolts = rightAppliedVolts_;
    inputs.rightCurrentAmps = {sim_.GetRightCurrentDraw().value()};
}

void DriveIOSim::SetVoltage(double leftVolts, double rightVolts) {
    closedLoop_ = false;
    leftAppliedVolts_ = leftVolts;
    rightAppliedVolts_ = rightVolts;
}

void DriveIOSim::SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop_ = true;
    leftFFVolts_ = leftFFVolts;
    rightFFVolts_ = rightFFVolts;
    leftPID_.SetSetpoint(leftRadPerSec);
    rightPID_.SetSetpoint(rightRadPerSec);
}
