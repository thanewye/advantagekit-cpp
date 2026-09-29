#pragma once

#include "akit/log/LogTable.h"

namespace akit {
    class LoggedDriverStation {
    public:
        static void SaveToLog(const LogTable& table);
        static void ReplayFromLog(const LogTable& table);
    };
} // namespace akit
