// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "Constants.h"
#include "commands/DriveCommands.h"
#include "subsystems/drive/DriveIOSim.h"
#include "subsystems/drive/DriveIOTalonSRX.h"
#include "subsystems/drive/GyroIOPigeon2.h"
#include "subsystems/superstructure/SuperstructureIOSim.h"
#include "subsystems/superstructure/SuperstructureIOTalonSRX.h"

RobotContainer::RobotContainer() {
    switch (Constants::GetCurrentMode()) {
        case Constants::Mode::kReal:
            // Real robot, instantiate hardware IO implementations
            drive_ = std::make_unique<Drive>(std::make_unique<DriveIOTalonSRX>(), std::make_unique<GyroIOPigeon2>());
            superstructure_ = std::make_unique<Superstructure>(std::make_unique<SuperstructureIOTalonSRX>());
            break;

        case Constants::Mode::kSim:
            // Sim robot, instantiate physics sim IO implementations
            drive_ = std::make_unique<Drive>(std::make_unique<DriveIOSim>(), std::make_unique<GyroIO>());
            superstructure_ = std::make_unique<Superstructure>(std::make_unique<SuperstructureIOSim>());
            break;

        default:
            // Replayed robot, disable IO implementations
            drive_ = std::make_unique<Drive>(std::make_unique<DriveIO>(), std::make_unique<GyroIO>());
            superstructure_ = std::make_unique<Superstructure>(std::make_unique<SuperstructureIO>());
            break;
    }

    // Set up auto routines
    pathplanner::NamedCommands::registerCommand("Launch", superstructure_->Launch().WithTimeout(6_s));
    AddAutoOption("None", frc2::cmd::None(), true);
    for (const auto& autoName : pathplanner::AutoBuilder::getAllAutoNames()) {
        AddAutoOption(autoName, pathplanner::PathPlannerAuto(autoName).ToPtr());
    }

    // Set up SysId routines
    AddAutoOption("Drive Simple FF Characterization", DriveCommands::FeedforwardCharacterization(drive_.get()));
    AddAutoOption("Drive SysId (Quasistatic Forward)", drive_->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    AddAutoOption("Drive SysId (Quasistatic Reverse)", drive_->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    AddAutoOption("Drive SysId (Dynamic Forward)", drive_->SysIdDynamic(frc2::sysid::Direction::kForward));
    AddAutoOption("Drive SysId (Dynamic Reverse)", drive_->SysIdDynamic(frc2::sysid::Direction::kReverse));

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Default drive command, normal arcade drive
    drive_->SetDefaultCommand(
        DriveCommands::ArcadeDrive(drive_.get(), [this] { return -controller_.GetLeftY(); }, [this] { return -controller_.GetRightX(); }));

    // Control bindings for superstructure
    controller_.LeftBumper().WhileTrue(superstructure_->Intake());
    controller_.RightBumper().WhileTrue(superstructure_->Launch());
    controller_.A().WhileTrue(superstructure_->Eject());
}

void RobotContainer::AddAutoOption(std::string_view name, frc2::CommandPtr command, bool isDefault) {
    if (isDefault) {
        autoChooser_.AddDefaultOption(name, command.get());
    } else {
        autoChooser_.AddOption(name, command.get());
    }
    autoOptions_.push_back(std::move(command));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoChooser_.Get();
}
