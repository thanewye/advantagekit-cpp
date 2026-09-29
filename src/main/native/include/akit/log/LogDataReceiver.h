#pragma once

#include <string_view>

#include "akit/log/LogTable.h"

namespace akit {
    class LogDataReceiver {
    public:
        static constexpr std::string_view kTimestampKey = "/Timestamp";

        virtual void Start() {}
        virtual void End() {}
        virtual ~LogDataReceiver() = default;
        virtual void PutTable(const LogTable& table) = 0;
    };
} // namespace akit
