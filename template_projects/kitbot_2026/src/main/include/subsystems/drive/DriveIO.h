// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <vector>

struct DriveIOInputs {
    double leftPositionRad = 0.0;
    double leftVelocityRadPerSec = 0.0;
    double leftAppliedVolts = 0.0;
    std::vector<double> leftCurrentAmps{};

    double rightPositionRad = 0.0;
    double rightVelocityRadPerSec = 0.0;
    double rightAppliedVolts = 0.0;
    std::vector<double> rightCurrentAmps{};
};

class DriveIO {
public:
    virtual ~DriveIO() = default;

    /** Updates the set of loggable inputs. */
    virtual void UpdateInputs(DriveIOInputs& inputs) {}

    /** Run open loop at the specified voltage. */
    virtual void SetVoltage(double leftVolts, double rightVolts) {}

    /** Run closed loop at the specified velocity. */
    virtual void SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}
};
