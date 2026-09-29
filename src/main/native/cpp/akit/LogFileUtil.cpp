#include "pch.h"

#include "akit/LogFileUtil.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>

namespace akit::LogFileUtil {
    namespace {
        constexpr std::string_view kAdvantageScopeFileName = "akit-log-path.txt";
    } // namespace

    std::string AddPathSuffix(const std::string& path, const std::string& suffix) {
        const size_t dotIndex = path.find_last_of('.');
        if (dotIndex == std::string::npos) return path;
        const std::string basename = path.substr(0, dotIndex);
        const std::string extension = path.substr(dotIndex);
        if (basename.ends_with(suffix)) return basename + "_2" + extension;
        if (std::regex_match(basename, std::regex(".+" + suffix + "_[0-9]+$"))) {
            const size_t splitIndex = basename.find_last_of('_');
            const int index = std::stoi(basename.substr(splitIndex + 1));
            return basename.substr(0, splitIndex) + "_" + std::to_string(index + 1) + extension;
        }
        return basename + suffix + extension;
    }

    std::string FindReplayLog() {
        if (const auto envPath = FindReplayLogEnvVar()) {
            std::cout << "[AdvantageKit] Replaying log from " << kEnvironmentVariable << " environment variable: \"" << *envPath << "\"\n";
            return *envPath;
        }

        if (const auto advantageScopeLogPath = FindReplayLogAdvantageScope()) {
            std::cout << "[AdvantageKit] Replaying log from AdvantageScope: \"" << *advantageScopeLogPath << "\"\n";
            return *advantageScopeLogPath;
        }

        std::cout << "No log provided with the " << kEnvironmentVariable
                  << " environment variable or through AdvantageScope. Enter path to file: " << std::flush;
        std::string filename = FindReplayLogUser();
        if (filename.size() >= 2 && (filename.front() == '\'' || filename.front() == '"')) filename = filename.substr(1, filename.size() - 2);
        return filename;
    }

    std::optional<std::string> FindReplayLogEnvVar() {
        const char* envPath = std::getenv(kEnvironmentVariable.data());
        if (envPath == nullptr || envPath[0] == '\0') return std::nullopt;
        return std::string(envPath);
    }

    std::optional<std::string> FindReplayLogAdvantageScope() {
        std::error_code ec;
        const auto tempDirectory = std::filesystem::temp_directory_path(ec);
        if (ec) return std::nullopt;

        std::ifstream file(tempDirectory / kAdvantageScopeFileName);
        std::string advantageScopeLogPath;
        if (!std::getline(file, advantageScopeLogPath)) return std::nullopt;
        if (advantageScopeLogPath.ends_with('\r')) advantageScopeLogPath.pop_back();
        if (advantageScopeLogPath.empty()) return std::nullopt;
        return advantageScopeLogPath;
    }

    std::string FindReplayLogUser() {
        std::string filename;
        std::getline(std::cin, filename);
        return filename;
    }
} // namespace akit::LogFileUtil
