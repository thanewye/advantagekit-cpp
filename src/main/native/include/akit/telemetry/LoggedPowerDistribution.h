#pragma once

#include <memory>
#include <optional>

#include <frc/PowerDistribution.h>
#include <hal/PowerDistribution.h>

#include "akit/log/LogTable.h"

namespace akit {
    /** Logs power distribution data; a missing module is skipped with a single warning instead of per-cycle errors. */
    class LoggedPowerDistribution {
    public:
        static LoggedPowerDistribution* GetInstance();
        static LoggedPowerDistribution* GetInstance(int moduleID, frc::PowerDistribution::ModuleType moduleType);

        /** Destroys the singleton instance, if any, releasing its HAL resources deterministically. */
        static void Reset();

        LoggedPowerDistribution(const LoggedPowerDistribution&) = delete;
        LoggedPowerDistribution& operator=(const LoggedPowerDistribution&) = delete;
        ~LoggedPowerDistribution();

        void SaveToLog(LogTable table) const;

    private:
        LoggedPowerDistribution();
        LoggedPowerDistribution(int moduleID, frc::PowerDistribution::ModuleType moduleType);

        int moduleID_;
        std::optional<frc::PowerDistribution::ModuleType> moduleType_;
        HAL_PowerDistributionHandle handle_;
        mutable bool disconnectedWarningReported_ = false;

        inline static std::unique_ptr<LoggedPowerDistribution> instance_;
    };
} // namespace akit
