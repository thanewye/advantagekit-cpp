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

#include <akit/networktables/LoggedNetworkChooser.h>
#include <wpi/commands2/Command.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/button/CommandXboxController.hpp>

#include "subsystems/drive/Drive.h"

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
    wpi::cmd::Command* GetAutonomousCommand();

private:
    /** Use this method to define your button->command mappings. */
    void ConfigureButtonBindings();

    /** Adds a command to the auto chooser and keeps it alive for the life of the container. */
    void AddAutoOption(std::string_view name, wpi::cmd::CommandPtr command, bool isDefault = false);

    // Subsystems
    std::unique_ptr<Drive> drive_;

    // Controller
    wpi::cmd::CommandXboxController controller_{0};

    // Dashboard inputs
    akit::networktables::LoggedNetworkChooser<wpi::cmd::Command*> autoChooser_{"Auto Choices"};
    std::vector<wpi::cmd::CommandPtr> autoOptions_;
};
