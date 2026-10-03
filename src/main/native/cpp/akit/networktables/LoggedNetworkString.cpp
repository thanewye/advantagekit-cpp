#include "pch.h"

#include "akit/networktables/LoggedNetworkString.h"

#include <wpi/nt/NetworkTableInstance.hpp>

#include "akit/Logger.h"

namespace akit::networktables {
    LoggedNetworkString::LoggedNetworkString(const std::string_view key)
        : key_(key)
        , entry_(wpi::nt::NetworkTableInstance::GetDefault().GetStringTopic(key).GetEntry(""))
        , value_(defaultValue_) {
        Logger::RegisterDashboardInput(this);
    }

    LoggedNetworkString::LoggedNetworkString(const std::string_view key, const std::string_view defaultValue)
        : LoggedNetworkString(key) {
        SetDefault(defaultValue);
        value_ = defaultValue_;
    }

    void LoggedNetworkString::SetDefault(const std::string_view defaultValue) {
        defaultValue_ = std::string(defaultValue);
        entry_.Set(entry_.Get(defaultValue_));
    }

    void LoggedNetworkString::Set(const std::string_view value) {
        entry_.Set(value);
    }

    void LoggedNetworkString::Periodic() {
        if (!Logger::HasReplaySource()) value_ = entry_.Get(defaultValue_);
        Logger::ProcessDashboardInput(kPrefix, RemoveSlash(key_), value_, defaultValue_);
    }
} // namespace akit::networktables
