#pragma once

#include <unordered_set>

#include "akit/log/LogTable.h"

namespace akit {
    class LoggedSystemStats {
    public:
        static void SaveToLog(LogTable table);

    private:
        inline static std::unordered_set<std::string> lastNTRemoteIds_;
    };
} // namespace akit
