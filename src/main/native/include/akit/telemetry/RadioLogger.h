#pragma once

#include <memory>
#include <mutex>
#include <regex>
#include <string>

#include <frc/Notifier.h>

#include "akit/log/LogTable.h"

namespace akit {
    class RadioLogger {
    public:
        static void Periodic(LogTable table);
        static void Stop();

    private:
        static void Start();
        static std::mutex mutex_;
        static bool isConnected_;
        static std::string statusJson_;
        static std::string statusURL_;
        static constexpr int kConnectTimeoutSecs = 1;
        static constexpr int kReadTimeoutSecs = 1;
        static constexpr int kRequestPeriodSecs = 5;
        static const std::regex kWhitespacePattern;
        static std::unique_ptr<frc::Notifier> notifier_;
    };
} // namespace akit
