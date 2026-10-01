// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandGenericHID.h>

#include "subsystems/drive/DemoDrive.h"
#include "subsystems/vision/Vision.h"

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    RobotContainer();

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    frc2::Command* GetAutonomousCommand();

private:
    /** Use this method to define your button->command mappings. */
    void ConfigureButtonBindings();

    std::unique_ptr<Vision> vision_;

    DemoDrive drive_;                       // Demo drive subsystem, sim only
    frc2::CommandGenericHID keyboard_{0}; // Keyboard 0 on port 0

    frc2::CommandPtr autonomousCommand_;
};
