// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>

#include <wpi/commands2/CommandPtr.hpp>

#include "subsystems/drive/Drive.h"

namespace DriveCommands {
    /**
     * Standard joystick drive, where X is the forward-backward axis (positive = forward) and Z is the
     * left-right axis (positive = counter-clockwise).
     */
    wpi::cmd::CommandPtr ArcadeDrive(Drive* drive, std::function<double()> xSupplier, std::function<double()> zSupplier);

    /** Measures the velocity feedforward constants for the drive. */
    wpi::cmd::CommandPtr FeedforwardCharacterization(Drive* drive);
} // namespace DriveCommands
