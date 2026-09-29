#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <wpi/DataLogWriter.h>

#include "akit/log/LogDataReceiver.h"
#include "akit/log/LoggableType.h"

namespace akit::wpilog {
    class WPILOGWriter : public LogDataReceiver {
    public:
        enum class AdvantageScopeOpenBehavior { kAlways, kAuto, kNever };

    private:
        static constexpr double kTimestampUpdateDelay = 5.0;
        static constexpr const char* kDefaultPathRio = "/U/logs";
        static constexpr const char* kDefaultPathSim = "logs";
        static constexpr const char* kAscopeFileName = "ascope-log-path.txt";

        std::string folder_;
        std::string fileName_;
        std::string randomIdentifier_;
        std::optional<double> dsAttachedTime_;
        static constexpr auto kTimeFormat = "%y-%m-%d_%H-%M-%S";

        bool autoRename_;
        std::optional<std::tm> logDate_;
        std::optional<std::string> logMatchText_;

        std::unique_ptr<wpi::log::DataLogWriter> log_;
        bool isOpen_ = false;
        AdvantageScopeOpenBehavior openBehavior_;

        LogStorage lastStorage_;
        int64_t timestampID_ = 0;

        std::unordered_map<std::string, int64_t> entryIDs_;
        std::unordered_map<std::string, LoggableType> entryTypes_;
        std::unordered_map<std::string, std::optional<std::string>> entryUnits_;

        [[nodiscard]] LoggableType GetType(const LogValue& value) const;
        void AppendValue(int64_t entryID, const LogValue& value, int64_t timestamp);

    public:
        WPILOGWriter(const std::string& path, AdvantageScopeOpenBehavior openBehavior);
        explicit WPILOGWriter(const std::string& path);
        explicit WPILOGWriter(AdvantageScopeOpenBehavior openBehavior);
        WPILOGWriter();

        void Start() override;
        void End() override;
        void PutTable(const LogTable& table) override;
    };
} // namespace akit::wpilog
