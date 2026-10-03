#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <wpi/datalog/DataLogReader.hpp>

#include "akit/log/LogReplaySource.h"
#include "akit/log/LoggableType.h"

namespace akit::wpilog {
    class WPILOGReader : public LogReplaySource {
    public:
        explicit WPILOGReader(std::string filename);

        void Start() override;
        bool UpdateTable(LogTable& table) override;

    private:
        std::string filename_;
        bool isValid_ = false;

        std::unique_ptr<wpi::log::DataLogReader> reader_;
        std::optional<wpi::log::DataLogIterator> iterator_;

        std::optional<int64_t> timestamp_;
        std::unordered_map<int, std::string> entryIDs_;
        std::unordered_map<int, LoggableType> entryTypes_;
        std::unordered_map<int, std::string> entryCustomTypes_;
        std::unordered_map<int, std::string> entryUnits_;
    };
} // namespace akit::wpilog
