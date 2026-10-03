#pragma once

#include <memory>

#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/hardware/power/PowerDistribution.hpp>

#include "akit/log/LogTable.h"

namespace akit {
    class LoggedPowerDistribution {
    public:
        /** Returns the configured instance, or nullptr if GetInstance(busId, moduleID, moduleType) has not been called. */
        static LoggedPowerDistribution* GetInstance();
        static LoggedPowerDistribution* GetInstance(wpi::CANPort busId, int moduleID, wpi::PowerDistribution::ModuleType moduleType);

        /** Destroys the singleton instance, if any, releasing its HAL resources deterministically. */
        static void Reset();

        void SaveToLog(LogTable table) const;

    private:
        LoggedPowerDistribution(wpi::CANPort busId, int moduleID, wpi::PowerDistribution::ModuleType moduleType);

        wpi::CANPort busId_;
        int moduleID_;
        wpi::PowerDistribution::ModuleType moduleType_;
        std::unique_ptr<wpi::PowerDistribution> powerDistribution_;

        static std::unique_ptr<LoggedPowerDistribution> instance_;
    };
} // namespace akit
