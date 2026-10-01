// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>

#include <ctre/phoenix/ErrorCode.h>
#include <ctre/phoenix/StatusCodes.h>

namespace PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    inline void TryUntilOk(int maxAttempts, const std::function<ctre::phoenix::StatusCode()>& command) {
        for (int i = 0; i < maxAttempts; i++) {
            auto error = command();
            if (error.IsOK()) break;
        }
    }

    /** Attempts to run the command until no error is produced. */
    inline void TryUntilOkV5(int maxAttempts, const std::function<ctre::phoenix::ErrorCode()>& command) {
        for (int i = 0; i < maxAttempts; i++) {
            auto error = command();
            if (error == ctre::phoenix::ErrorCode::OK) break;
        }
    }
} // namespace PhoenixUtil
