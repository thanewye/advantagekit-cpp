#include "pch.h"

#include "akit/networktables/LoggedNetworkInput.h"

#include "akit/Logger.h"

namespace akit::networktables {
    LoggedNetworkInput::~LoggedNetworkInput() {
        Logger::UnregisterDashboardInput(this);
    }

    std::string LoggedNetworkInput::RemoveSlash(const std::string_view key) {
        if (!key.empty() && key.front() == '/') return std::string(key.substr(1));
        return std::string(key);
    }
} // namespace akit::networktables
