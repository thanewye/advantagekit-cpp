// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include <akit/networktables/LoggedDashboardChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/drive/Drive.h"
#include "subsystems/superstructure/Superstructure.h"

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

    /** Adds a command to the auto chooser and keeps it alive for the life of the container. */
    void AddAutoOption(std::string_view name, frc2::CommandPtr command, bool isDefault = false);

    // Subsystems
    std::unique_ptr<Drive> drive_;
    std::unique_ptr<Superstructure> superstructure_;

    // Controller
    frc2::CommandXboxController controller_{0};

    // Dashboard inputs
    akit::networktables::LoggedDashboardChooser<frc2::Command*> autoChooser_{"Auto Choices"};
    std::vector<frc2::CommandPtr> autoOptions_;
};
