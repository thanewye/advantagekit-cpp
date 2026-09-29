#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace akit::LogFileUtil {
    inline constexpr std::string_view kEnvironmentVariable = "AKIT_LOG_PATH";

    std::string AddPathSuffix(const std::string& path, const std::string& suffix);

    std::string FindReplayLog();

    std::optional<std::string> FindReplayLogEnvVar();

    std::optional<std::string> FindReplayLogAdvantageScope();

    std::string FindReplayLogUser();
} // namespace akit::LogFileUtil
