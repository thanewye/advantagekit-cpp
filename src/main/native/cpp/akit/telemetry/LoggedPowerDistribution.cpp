#include "pch.h"

#include "akit/telemetry/LoggedPowerDistribution.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace akit {
    std::unique_ptr<LoggedPowerDistribution> LoggedPowerDistribution::instance_;

    LoggedPowerDistribution::LoggedPowerDistribution(const wpi::CANPort busId, const int moduleID, const wpi::PowerDistribution::ModuleType moduleType)
        : busId_(busId)
        , moduleID_(moduleID)
        , moduleType_(moduleType)
        , powerDistribution_(std::make_unique<wpi::PowerDistribution>(busId, moduleID, moduleType)) {}

    LoggedPowerDistribution* LoggedPowerDistribution::GetInstance() {
        return instance_.get();
    }

    LoggedPowerDistribution* LoggedPowerDistribution::GetInstance(const wpi::CANPort busId, const int moduleID,
                                                                  const wpi::PowerDistribution::ModuleType moduleType) {
        if (!instance_ || instance_->busId_ != busId || instance_->moduleID_ != moduleID || instance_->moduleType_ != moduleType) {
            instance_.reset();
            instance_ = std::unique_ptr<LoggedPowerDistribution>(new LoggedPowerDistribution(busId, moduleID, moduleType));
        }
        return instance_.get();
    }

    void LoggedPowerDistribution::Reset() {
        instance_.reset();
    }

    void LoggedPowerDistribution::SaveToLog(LogTable table) const {
        table.Put("Temperature", powerDistribution_->GetTemperature());
        table.Put("Voltage", powerDistribution_->GetVoltage());

        std::array<double, 24> channelCurrents{};
        const auto allCurrents = powerDistribution_->GetAllCurrents();
        std::copy_n(allCurrents.begin(), std::min(channelCurrents.size(), allCurrents.size()), channelCurrents.begin());
        table.Put("ChannelCurrent", std::span<const double>(channelCurrents));

        table.Put("TotalCurrent", powerDistribution_->GetTotalCurrent());
        table.Put("TotalPower", powerDistribution_->GetTotalPower());
        table.Put("TotalEnergy", powerDistribution_->GetTotalEnergy());
        table.Put("ChannelCount", static_cast<int64_t>(powerDistribution_->GetNumChannels()));
        auto bitfieldToInteger = []<typename T>(const T& bitfield) {
            uint32_t raw = 0;
            static_assert(sizeof(raw) >= sizeof(T));
            std::memcpy(&raw, &bitfield, sizeof(T));
            return static_cast<int64_t>(raw);
        };
        table.Put("Faults", bitfieldToInteger(powerDistribution_->GetFaults()));
        table.Put("StickyFaults", bitfieldToInteger(powerDistribution_->GetStickyFaults()));
    }
} // namespace akit
