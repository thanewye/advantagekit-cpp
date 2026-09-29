#include "pch.h"

#include "akit/telemetry/LoggedPowerDistribution.h"

#include <algorithm>
#include <array>

namespace akit {
    LoggedPowerDistribution::LoggedPowerDistribution()
        : moduleID_(frc::PowerDistribution::kDefaultModule)
        , powerDistribution_(std::make_unique<frc::PowerDistribution>()) {}

    LoggedPowerDistribution::LoggedPowerDistribution(const int moduleID, const frc::PowerDistribution::ModuleType moduleType)
        : moduleID_(moduleID)
        , moduleType_(moduleType)
        , powerDistribution_(std::make_unique<frc::PowerDistribution>(moduleID, moduleType)) {}

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
