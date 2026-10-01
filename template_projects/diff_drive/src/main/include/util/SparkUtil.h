// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>
#include <span>
#include <vector>

#include <rev/REVLibError.h>
#include <rev/SparkBase.h>

namespace SparkUtil {
    /** Stores whether any error was has been detected by other utility methods. */
    inline bool sparkStickyFault = false;

    /** Processes a value from a Spark only if the value is valid. */
    inline void IfOk(rev::spark::SparkBase& spark, const std::function<double()>& supplier, const std::function<void(double)>& consumer) {
        double value = supplier();
        if (spark.GetLastError() == rev::REVLibError::kOk) {
            consumer(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /** Processes a value from a Spark only if the value is valid. */
    inline void IfOk(rev::spark::SparkBase& spark, std::span<const std::function<double()>> suppliers,
                     const std::function<void(const std::vector<double>&)>& consumer) {
        std::vector<double> values(suppliers.size());
        for (size_t i = 0; i < suppliers.size(); i++) {
            values[i] = suppliers[i]();
            if (spark.GetLastError() != rev::REVLibError::kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer(values);
    }

    /** Attempts to run the command until no error is produced. */
    inline void TryUntilOk(rev::spark::SparkBase& spark, int maxAttempts, const std::function<rev::REVLibError()>& command) {
        for (int i = 0; i < maxAttempts; i++) {
            auto error = command();
            if (error == rev::REVLibError::kOk) {
                break;
            } else {
                sparkStickyFault = true;
            }
        }
    }
} // namespace SparkUtil
