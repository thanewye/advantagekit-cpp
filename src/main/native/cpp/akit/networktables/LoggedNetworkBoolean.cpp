#include "pch.h"

#include "akit/networktables/LoggedNetworkBoolean.h"

#include <networktables/NetworkTableInstance.h>

#include "akit/Logger.h"

namespace akit::networktables {
    LoggedNetworkBoolean::LoggedNetworkBoolean(const std::string_view key)
        : key_(key)
        , entry_(nt::NetworkTableInstance::GetDefault().GetBooleanTopic(key).GetEntry(false))
        , value_(defaultValue_) {
        Logger::RegisterDashboardInput(this);
    }

    LoggedNetworkBoolean::LoggedNetworkBoolean(const std::string_view key, const bool defaultValue)
        : LoggedNetworkBoolean(key) {
        SetDefault(defaultValue);
    }

    void LoggedNetworkBoolean::SetDefault(const bool defaultValue) {
        defaultValue_ = defaultValue;
        entry_.Set(entry_.Get(defaultValue));
    }

    void LoggedNetworkBoolean::Set(const bool value) {
        entry_.Set(value);
    }

    void LoggedNetworkBoolean::Periodic() {
        if (!Logger::HasReplaySource()) value_ = entry_.Get(defaultValue_);
        Logger::ProcessDashboardInput(kPrefix, RemoveSlash(key_), value_, defaultValue_);
    }
} // namespace akit::networktables
