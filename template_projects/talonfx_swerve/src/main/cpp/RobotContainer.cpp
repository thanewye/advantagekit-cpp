// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "RobotContainer.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "Constants.h"
#include "commands/DriveCommands.h"
#include "generated/TunerConstants.h"
#include "subsystems/drive/GyroIOPigeon2.h"
#include "subsystems/drive/ModuleIOSim.h"
#include "subsystems/drive/ModuleIOTalonFX.h"

RobotContainer::RobotContainer() {
    switch (Constants::GetCurrentMode()) {
        case Constants::Mode::kReal:
            // Real robot, instantiate hardware IO implementations
            // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
            // a CANcoder
            drive_ = std::make_unique<Drive>(std::make_unique<GyroIOPigeon2>(), std::make_unique<ModuleIOTalonFX>(TunerConstants::FrontLeft),
                                             std::make_unique<ModuleIOTalonFX>(TunerConstants::FrontRight),
                                             std::make_unique<ModuleIOTalonFX>(TunerConstants::BackLeft),
                                             std::make_unique<ModuleIOTalonFX>(TunerConstants::BackRight));

            // The ModuleIOTalonFXS implementation provides an example implementation for
            // TalonFXS controller connected to a CANdi with a PWM encoder. The
            // implementations of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from
            // the Spark swerve template) can be freely intermixed to support alternative
            // hardware arrangements.
            // Please see the AdvantageKit template documentation for more information:
            // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
            //
            // drive_ = std::make_unique<Drive>(std::make_unique<GyroIOPigeon2>(),
            //                                  std::make_unique<ModuleIOTalonFXS>(TunerConstants::FrontLeft),
            //                                  std::make_unique<ModuleIOTalonFXS>(TunerConstants::FrontRight),
            //                                  std::make_unique<ModuleIOTalonFXS>(TunerConstants::BackLeft),
            //                                  std::make_unique<ModuleIOTalonFXS>(TunerConstants::BackRight));
            break;

        case Constants::Mode::kSim:
            // Sim robot, instantiate physics sim IO implementations
            drive_ = std::make_unique<Drive>(std::make_unique<GyroIO>(), std::make_unique<ModuleIOSim>(TunerConstants::FrontLeft),
                                             std::make_unique<ModuleIOSim>(TunerConstants::FrontRight),
                                             std::make_unique<ModuleIOSim>(TunerConstants::BackLeft),
                                             std::make_unique<ModuleIOSim>(TunerConstants::BackRight));
            break;

        default:
            // Replayed robot, disable IO implementations
            drive_ = std::make_unique<Drive>(std::make_unique<GyroIO>(), std::make_unique<ModuleIO>(), std::make_unique<ModuleIO>(),
                                             std::make_unique<ModuleIO>(), std::make_unique<ModuleIO>());
            break;
    }

    // Set up auto routines
    AddAutoOption("None", frc2::cmd::None(), true);
    for (const auto& autoName : pathplanner::AutoBuilder::getAllAutoNames()) {
        AddAutoOption(autoName, pathplanner::PathPlannerAuto(autoName).ToPtr());
    }

    // Set up SysId routines
    AddAutoOption("Drive Wheel Radius Characterization", DriveCommands::WheelRadiusCharacterization(drive_.get()));
    AddAutoOption("Drive Simple FF Characterization", DriveCommands::FeedforwardCharacterization(drive_.get()));
    AddAutoOption("Drive SysId (Quasistatic Forward)", drive_->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    AddAutoOption("Drive SysId (Quasistatic Reverse)", drive_->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    AddAutoOption("Drive SysId (Dynamic Forward)", drive_->SysIdDynamic(frc2::sysid::Direction::kForward));
    AddAutoOption("Drive SysId (Dynamic Reverse)", drive_->SysIdDynamic(frc2::sysid::Direction::kReverse));

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Default command, normal field-relative drive
    drive_->SetDefaultCommand(DriveCommands::JoystickDrive(
        drive_.get(), [this] { return -controller_.GetLeftY(); }, [this] { return -controller_.GetLeftX(); },
        [this] { return -controller_.GetRightX(); }));

    // Lock to 0° when A button is held
    controller_.A().WhileTrue(DriveCommands::JoystickDriveAtAngle(
        drive_.get(), [this] { return -controller_.GetLeftY(); }, [this] { return -controller_.GetLeftX(); }, [] { return frc::Rotation2d{}; }));

    // Switch to X pattern when X button is pressed
    controller_.X().OnTrue(frc2::cmd::RunOnce([this] { drive_->StopWithX(); }, {drive_.get()}));

    // Reset gyro to 0° when B button is pressed
    controller_.B().OnTrue(
        frc2::cmd::RunOnce([this] { drive_->SetPose(frc::Pose2d{drive_->GetPose().Translation(), frc::Rotation2d{}}); }, {drive_.get()})
            .IgnoringDisable(true));
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
