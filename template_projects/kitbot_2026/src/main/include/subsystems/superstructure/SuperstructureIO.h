// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

struct SuperstructureIOInputs {
    double feederPositionRad = 0.0;
    double feederVelocityRadPerSec = 0.0;
    double feederAppliedVolts = 0.0;
    double feederCurrentAmps = 0.0;

    double intakeLauncherPositionRad = 0.0;
    double intakeLauncherVelocityRadPerSec = 0.0;
    double intakeLauncherAppliedVolts = 0.0;
    double intakeLauncherCurrentAmps = 0.0;
};

class SuperstructureIO {
public:
    virtual ~SuperstructureIO() = default;

    /** Update the set of loggable inputs. */
    virtual void UpdateInputs(SuperstructureIOInputs& inputs) {}

    /** Run the feeder at the specified voltage. */
    virtual void SetFeederVoltage(double volts) {}

    /** Run the intake and launcher at the specified voltage. */
    virtual void SetIntakeLauncherVoltage(double volts) {}
};
