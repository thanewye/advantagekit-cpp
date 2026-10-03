// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "RobotContainer.h"

#include <memory>
#include <numbers>

#include <wpi/commands2/Commands.hpp>
#include <wpi/math/controller/PIDController.hpp>

#include "Constants.h"
#include "subsystems/vision/VisionConstants.h"
#include "subsystems/vision/VisionIOLimelight.h"
#include "subsystems/vision/VisionIOPhotonVisionSim.h"

using namespace VisionConstants;

RobotContainer::RobotContainer()
    : autonomousCommand_(wpi::cmd::None()) {
    auto addVisionMeasurement = [this](const wpi::math::Pose2d& visionRobotPoseMeters, wpi::units::second_t timestampSeconds,
                                       const wpi::util::array<double, 3>& visionMeasurementStdDevs) {
        drive_.AddVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    };

    switch (Constants::GetCurrentMode()) {
    case Constants::Mode::kReal:
        // Real robot, instantiate hardware IO implementations
        vision_ = std::make_unique<Vision>(addVisionMeasurement, std::make_unique<VisionIOLimelight>(camera0Name, [this] { return drive_.GetRotation(); }),
                                           std::make_unique<VisionIOLimelight>(camera1Name, [this] { return drive_.GetRotation(); }));
        // vision_ = std::make_unique<Vision>(addVisionMeasurement,
        //                                    std::make_unique<VisionIOPhotonVision>(camera0Name, robotToCamera0),
        //                                    std::make_unique<VisionIOPhotonVision>(camera1Name, robotToCamera1));
        break;

    case Constants::Mode::kSim:
        // Sim robot, instantiate physics sim IO implementations
        vision_ = std::make_unique<Vision>(addVisionMeasurement,
                                           std::make_unique<VisionIOPhotonVisionSim>(camera0Name, robotToCamera0, [this] { return drive_.GetPose(); }),
                                           std::make_unique<VisionIOPhotonVisionSim>(camera1Name, robotToCamera1, [this] { return drive_.GetPose(); }));
        break;

    default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision_ = std::make_unique<Vision>(addVisionMeasurement, std::make_unique<VisionIO>(), std::make_unique<VisionIO>());
        break;
    }

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Joystick drive command
    drive_.SetDefaultCommand(wpi::cmd::Run([this] { drive_.Run(-keyboard_.GetHID().GetRawAxis(1), -keyboard_.GetHID().GetRawAxis(0)); }, {&drive_}));

    // Auto aim command example
    auto aimController = std::make_shared<wpi::math::PIDController>(0.2, 0.0, 0.0);
    aimController->EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
    keyboard_.Button(1).WhileTrue(
        wpi::cmd::StartRun([aimController] { aimController->Reset(); },
                           [this, aimController] { drive_.Run(0.0, aimController->Calculate(vision_->GetTargetX(0).Radians().value())); }, {&drive_}));
}

wpi::cmd::Command* RobotContainer::GetAutonomousCommand() {
    return autonomousCommand_.get();
}
