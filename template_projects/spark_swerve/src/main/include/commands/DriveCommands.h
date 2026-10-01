// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/drive/Drive.h"

namespace DriveCommands {
    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    frc2::CommandPtr JoystickDrive(Drive* drive, std::function<double()> xSupplier, std::function<double()> ySupplier,
                                   std::function<double()> omegaSupplier);

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    frc2::CommandPtr JoystickDriveAtAngle(Drive* drive, std::function<double()> xSupplier, std::function<double()> ySupplier,
                                          std::function<frc::Rotation2d()> rotationSupplier);

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    frc2::CommandPtr FeedforwardCharacterization(Drive* drive);

    /** Measures the robot's wheel radius by spinning in a circle. */
    frc2::CommandPtr WheelRadiusCharacterization(Drive* drive);
} // namespace DriveCommands
