// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <wpi/framework/RobotBase.hpp>

/**
 * This namespace defines the runtime mode used by AdvantageKit. The mode is always "real" when
 * running on a Systemcore. Change the value of "kSimMode" to switch between "sim" (physics sim) and
 * "replay" (log replay from a file).
 */
namespace Constants {
    enum class Mode {
        /** Running on a real robot. */
        kReal,

        /** Running a physics simulator. */
        kSim,

        /** Replaying from a log file. */
        kReplay
    };

    inline constexpr Mode kSimMode = Mode::kSim;

    inline Mode GetCurrentMode() {
        return wpi::RobotBase::IsReal() ? Mode::kReal : kSimMode;
    }
} // namespace Constants
