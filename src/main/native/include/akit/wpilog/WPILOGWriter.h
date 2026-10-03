#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <wpi/datalog/DataLogWriter.hpp>

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

        struct EntryState {
            int64_t id = 0;
            std::optional<std::string> unit;
            std::optional<LogValue> lastWrittenValue;
            uint64_t lastPresentCycle = 0;
        };

        int64_t timestampID_ = 0;
        uint64_t cycle_ = 0;
        std::unordered_map<std::string, EntryState> entries_;

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
