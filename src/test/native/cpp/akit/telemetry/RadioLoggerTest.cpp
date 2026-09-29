#include "akit/telemetry/RadioLogger.h"
#include "gtest/gtest.h"

namespace {

    using akit::RadioLogger;

    TEST(RadioLoggerTest, StopIsSafeWhenNeverStarted) {
        RadioLogger::Stop();
    }

    TEST(RadioLoggerTest, StopIsSafeToCallRepeatedly) {
        RadioLogger::Stop();
        RadioLogger::Stop();
    }

} // namespace
