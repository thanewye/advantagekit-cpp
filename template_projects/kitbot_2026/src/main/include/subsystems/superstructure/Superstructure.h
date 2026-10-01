// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "subsystems/superstructure/SuperstructureIO.h"

class Superstructure : public frc2::SubsystemBase {
public:
    explicit Superstructure(std::unique_ptr<SuperstructureIO> io);

    void Periodic() override;

    /** Set the rollers to the values for intaking. */
    frc2::CommandPtr Intake();

    /** Set the rollers to the values for ejecting fuel out the intake. */
    frc2::CommandPtr Eject();

    /** Set the rollers to the values for launching. Spins up before feeding fuel. */
    frc2::CommandPtr Launch();

private:
    std::unique_ptr<SuperstructureIO> io_;
    SuperstructureIOInputs inputs_;
};
