#pragma once

#include "akit/log/LogTable.h"

namespace akit {
    class LogReplaySource {
    public:
        virtual ~LogReplaySource() = default;

        virtual void Start() {}
        virtual void End() {}
        virtual bool UpdateTable(LogTable& table) = 0;
    };
} // namespace akit
