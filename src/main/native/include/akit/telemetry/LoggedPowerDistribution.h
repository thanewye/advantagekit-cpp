#pragma once

#include <memory>
#include <optional>

#include <frc/PowerDistribution.h>

#include "akit/log/LogTable.h"

namespace akit {
    class LoggedPowerDistribution {
    public:
        static LoggedPowerDistribution* GetInstance();
        static LoggedPowerDistribution* GetInstance(int moduleID, frc::PowerDistribution::ModuleType moduleType);

        /** Destroys the singleton instance, if any, releasing its HAL resources deterministically. */
        static void Reset();

        void SaveToLog(LogTable table) const;

    private:
        LoggedPowerDistribution();
        LoggedPowerDistribution(int moduleID, frc::PowerDistribution::ModuleType moduleType);

        int moduleID_;
        std::optional<frc::PowerDistribution::ModuleType> moduleType_;
        std::unique_ptr<frc::PowerDistribution> powerDistribution_;

        inline static std::unique_ptr<LoggedPowerDistribution> instance_;
    };
} // namespace akit
