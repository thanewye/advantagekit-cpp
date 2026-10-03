#pragma once

#include <memory>
#include <mutex>
#include <regex>
#include <string>

#include <wpi/system/Notifier.hpp>

#include "akit/log/LogTable.h"

namespace akit {
    class RadioLogger {
    public:
        static void Periodic(LogTable table, int64_t teamNumber);
        static void Stop();

    private:
        static void Start(int64_t teamNumber);
        static std::mutex mutex_;
        static bool isConnected_;
        static std::string statusJson_;
        static std::string statusURL_;
        static constexpr int kConnectTimeoutSecs = 1;
        static constexpr int kReadTimeoutSecs = 1;
        static constexpr int kRequestPeriodSecs = 5;
        static const std::regex kWhitespacePattern;
        static std::unique_ptr<wpi::Notifier> notifier_;
    };
} // namespace akit
