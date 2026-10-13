#include "pch.h"

#include "akit/telemetry/LoggedPowerDistribution.h"

#include <algorithm>
#include <array>
#include <cstring>

#include <frc/Errors.h>
#include <wpi/StackTrace.h>

namespace akit {
    namespace {
        HAL_PowerDistributionHandle InitializePowerDistributionIgnoringStatus(const int moduleID, const HAL_PowerDistributionType moduleType) {
            int32_t ignoredStatus = 0;
            return HAL_InitializePowerDistribution(moduleID, moduleType, wpi::GetStackTrace(1).c_str(), &ignoredStatus);
        }
    } // namespace

    LoggedPowerDistribution::LoggedPowerDistribution()
        : moduleID_(frc::PowerDistribution::kDefaultModule)
        , handle_(InitializePowerDistributionIgnoringStatus(frc::PowerDistribution::kDefaultModule, HAL_PowerDistributionType_kAutomatic)) {}

    LoggedPowerDistribution::LoggedPowerDistribution(const int moduleID, const frc::PowerDistribution::ModuleType moduleType)
        : moduleID_(moduleID)
        , moduleType_(moduleType)
        , handle_(InitializePowerDistributionIgnoringStatus(moduleID, static_cast<HAL_PowerDistributionType>(moduleType))) {}

    LoggedPowerDistribution::~LoggedPowerDistribution() {
        HAL_CleanPowerDistribution(handle_);
    }

    LoggedPowerDistribution* LoggedPowerDistribution::GetInstance() {
        if (!instance_) {
            instance_ = std::unique_ptr<LoggedPowerDistribution>(new LoggedPowerDistribution());
        }
        return instance_.get();
    }

    LoggedPowerDistribution* LoggedPowerDistribution::GetInstance(const int moduleID, const frc::PowerDistribution::ModuleType moduleType) {
        if (!instance_ || instance_->moduleID_ != moduleID || instance_->moduleType_ != moduleType) {
            instance_ = std::unique_ptr<LoggedPowerDistribution>(new LoggedPowerDistribution(moduleID, moduleType));
        }
        return instance_.get();
    }

    void LoggedPowerDistribution::Reset() {
        instance_.reset();
    }

    void LoggedPowerDistribution::SaveToLog(LogTable table) const {
        int32_t connectionStatus = 0;
        const double voltage = HAL_GetPowerDistributionVoltage(handle_, &connectionStatus);
        if (connectionStatus != 0) {
            if (!disconnectedWarningReported_) {
                FRC_ReportError(frc::warn::Warning, "[AdvantageKit] Power distribution module {} not responding, skipping power distribution logging.", moduleID_);
                disconnectedWarningReported_ = true;
            }
            return;
        }

        int32_t ignoredStatus = 0;
        table.Put("Temperature", HAL_GetPowerDistributionTemperature(handle_, &ignoredStatus));
        table.Put("Voltage", voltage);

        const int32_t channelCount = HAL_GetPowerDistributionNumChannels(handle_, &ignoredStatus);
        std::array<double, 24> channelCurrents{};
        HAL_GetPowerDistributionAllChannelCurrents(handle_, channelCurrents.data(),
                                                   std::clamp(channelCount, 0, static_cast<int32_t>(channelCurrents.size())), &ignoredStatus);
        table.Put("ChannelCurrent", std::span<const double>(channelCurrents));

        table.Put("TotalCurrent", HAL_GetPowerDistributionTotalCurrent(handle_, &ignoredStatus));
        table.Put("TotalPower", HAL_GetPowerDistributionTotalPower(handle_, &ignoredStatus));
        table.Put("TotalEnergy", HAL_GetPowerDistributionTotalEnergy(handle_, &ignoredStatus));
        table.Put("ChannelCount", static_cast<int64_t>(channelCount));
        auto bitfieldToInteger = []<typename T>(const T& bitfield) {
            uint32_t raw = 0;
            static_assert(sizeof(raw) >= sizeof(T));
            std::memcpy(&raw, &bitfield, sizeof(T));
            return static_cast<int64_t>(raw);
        };
        HAL_PowerDistributionFaults faults{};
        HAL_GetPowerDistributionFaults(handle_, &faults, &ignoredStatus);
        table.Put("Faults", bitfieldToInteger(faults));
        HAL_PowerDistributionStickyFaults stickyFaults{};
        HAL_GetPowerDistributionStickyFaults(handle_, &stickyFaults, &ignoredStatus);
        table.Put("StickyFaults", bitfieldToInteger(stickyFaults));
    }
} // namespace akit
