// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

namespace SuperstructureConstants {
    inline constexpr int feederCanId = 5;
    inline constexpr double feederMotorReduction = 1.0;
    inline constexpr int feederCurrentLimit = 60;

    inline constexpr int intakeLauncherCanId = 6;
    inline constexpr double intakeLauncherMotorReduction = 1.0;
    inline constexpr int intakeLauncherCurrentLimit = 60;

    inline constexpr double intakingFeederVoltage = -12.0;
    inline constexpr double intakingIntakeVoltage = 10.0;
    inline constexpr double launchingFeederVoltage = 9.0;
    inline constexpr double launchingLauncherVoltage = 10.6;
    inline constexpr double spinUpFeederVoltage = -6.0;
    inline constexpr double spinUpSeconds = 1.0;
} // namespace SuperstructureConstants
