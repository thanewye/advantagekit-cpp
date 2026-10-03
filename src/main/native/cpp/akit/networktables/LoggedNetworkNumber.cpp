#include "pch.h"

#include "akit/networktables/LoggedNetworkNumber.h"

#include <wpi/nt/NetworkTableInstance.hpp>

#include "akit/Logger.h"

namespace akit::networktables {
    LoggedNetworkNumber::LoggedNetworkNumber(const std::string_view key)
        : key_(key)
        , entry_(wpi::nt::NetworkTableInstance::GetDefault().GetDoubleTopic(key).GetEntry(0.0))
        , value_(defaultValue_) {
        Logger::RegisterDashboardInput(this);
    }

    LoggedNetworkNumber::LoggedNetworkNumber(const std::string_view key, const double defaultValue)
        : LoggedNetworkNumber(key) {
        SetDefault(defaultValue);
        value_ = defaultValue;
    }

    void LoggedNetworkNumber::SetDefault(const double defaultValue) {
        defaultValue_ = defaultValue;
        entry_.Set(entry_.Get(defaultValue));
    }

    void LoggedNetworkNumber::Set(const double value) {
        entry_.Set(value);
    }

    void LoggedNetworkNumber::Periodic() {
        if (!Logger::HasReplaySource()) value_ = entry_.Get(defaultValue_);
        Logger::ProcessDashboardInput(kPrefix, RemoveSlash(key_), value_, defaultValue_);
    }
} // namespace akit::networktables
