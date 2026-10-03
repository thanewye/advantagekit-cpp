// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "RobotContainer.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <wpi/commands2/Commands.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>

#include "Constants.h"
#include "commands/DriveCommands.h"
#include "subsystems/drive/GyroIOPigeon2.h"
#include "subsystems/drive/ModuleIOSim.h"
#include "subsystems/drive/ModuleIOSpark.h"

RobotContainer::RobotContainer() {
    switch (Constants::GetCurrentMode()) {
    case Constants::Mode::kReal:
        // Real robot, instantiate hardware IO implementations
        drive_ = std::make_unique<Drive>(std::make_unique<GyroIOPigeon2>(), std::make_unique<ModuleIOSpark>(0), std::make_unique<ModuleIOSpark>(1),
                                         std::make_unique<ModuleIOSpark>(2), std::make_unique<ModuleIOSpark>(3));
        break;

    case Constants::Mode::kSim:
        // Sim robot, instantiate physics sim IO implementations
        drive_ = std::make_unique<Drive>(std::make_unique<GyroIO>(), std::make_unique<ModuleIOSim>(), std::make_unique<ModuleIOSim>(),
                                         std::make_unique<ModuleIOSim>(), std::make_unique<ModuleIOSim>());
        break;

    default:
        // Replayed robot, disable IO implementations
        drive_ = std::make_unique<Drive>(std::make_unique<GyroIO>(), std::make_unique<ModuleIO>(), std::make_unique<ModuleIO>(), std::make_unique<ModuleIO>(),
                                         std::make_unique<ModuleIO>());
        break;
    }

    // Set up auto routines
    AddAutoOption("None", wpi::cmd::None(), true);
    for (const auto& autoName : pathplanner::AutoBuilder::getAllAutoNames()) {
        AddAutoOption(autoName, pathplanner::PathPlannerAuto(autoName).ToPtr());
    }

    // Set up SysId routines
    AddAutoOption("Drive Wheel Radius Characterization", DriveCommands::WheelRadiusCharacterization(drive_.get()));
    AddAutoOption("Drive Simple FF Characterization", DriveCommands::FeedforwardCharacterization(drive_.get()));
    AddAutoOption("Drive SysId (Quasistatic Forward)", drive_->SysIdQuasistatic(wpi::cmd::sysid::Direction::FORWARD));
    AddAutoOption("Drive SysId (Quasistatic Reverse)", drive_->SysIdQuasistatic(wpi::cmd::sysid::Direction::REVERSE));
    AddAutoOption("Drive SysId (Dynamic Forward)", drive_->SysIdDynamic(wpi::cmd::sysid::Direction::FORWARD));
    AddAutoOption("Drive SysId (Dynamic Reverse)", drive_->SysIdDynamic(wpi::cmd::sysid::Direction::REVERSE));

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Default command, normal field-relative drive
    drive_->SetDefaultCommand(DriveCommands::JoystickDrive(
        drive_.get(), [this] { return -controller_.GetLeftY(); }, [this] { return -controller_.GetLeftX(); }, [this] { return -controller_.GetRightX(); }));

    // Lock to 0° when A button is held
    controller_.A().WhileTrue(DriveCommands::JoystickDriveAtAngle(
        drive_.get(), [this] { return -controller_.GetLeftY(); }, [this] { return -controller_.GetLeftX(); }, [] { return wpi::math::Rotation2d{}; }));

    // Switch to X pattern when X button is pressed
    controller_.X().OnTrue(wpi::cmd::RunOnce([this] { drive_->StopWithX(); }, {drive_.get()}));

    // Reset gyro to 0° when B button is pressed
    controller_.B().OnTrue(
        wpi::cmd::RunOnce([this] { drive_->SetPose(wpi::math::Pose2d{drive_->GetPose().Translation(), wpi::math::Rotation2d{}}); }, {drive_.get()})
            .IgnoringDisable(true));
}

void RobotContainer::AddAutoOption(std::string_view name, wpi::cmd::CommandPtr command, bool isDefault) {
    if (isDefault) {
        autoChooser_.AddDefaultOption(name, command.get());
    } else {
        autoChooser_.AddOption(name, command.get());
    }
    autoOptions_.push_back(std::move(command));
}

wpi::cmd::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser_.Get();
}
