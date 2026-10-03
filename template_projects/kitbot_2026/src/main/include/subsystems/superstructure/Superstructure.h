// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>

#include "subsystems/superstructure/SuperstructureIO.h"

class Superstructure : public wpi::cmd::SubsystemBase {
public:
    explicit Superstructure(std::unique_ptr<SuperstructureIO> io);

    void Periodic() override;

    /** Set the rollers to the values for intaking. */
    wpi::cmd::CommandPtr Intake();

    /** Set the rollers to the values for ejecting fuel out the intake. */
    wpi::cmd::CommandPtr Eject();

    /** Set the rollers to the values for launching. Spins up before feeding fuel. */
    wpi::cmd::CommandPtr Launch();

private:
    std::unique_ptr<SuperstructureIO> io_;
    SuperstructureIOInputs inputs_;
};
