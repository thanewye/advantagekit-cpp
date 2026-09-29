#include <cstdlib>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "akit/LogFileUtil.h"

namespace {
    namespace LogFileUtil = akit::LogFileUtil;

    void SetEnvironmentVariable(const std::string& value) {
        const std::string name(LogFileUtil::kEnvironmentVariable);
#ifdef _WIN32
        _putenv_s(name.c_str(), value.c_str());
#else
        setenv(name.c_str(), value.c_str(), 1);
#endif
    }

    void UnsetEnvironmentVariable() {
        const std::string name(LogFileUtil::kEnvironmentVariable);
#ifdef _WIN32
        _putenv_s(name.c_str(), "");
#else
        unsetenv(name.c_str());
#endif
    }

    class LogFileUtilEnvironmentTest : public ::testing::Test {
    protected:
        void SetUp() override {
            const char* original = std::getenv(std::string(LogFileUtil::kEnvironmentVariable).c_str());
            if (original != nullptr) originalValue_ = original;
        }

        void TearDown() override {
            if (originalValue_) SetEnvironmentVariable(*originalValue_);
            else UnsetEnvironmentVariable();
        }

    private:
        std::optional<std::string> originalValue_;
    };

    TEST(LogFileUtilTest, AddPathSuffixAppendsSuffixBeforeExtension) {
        EXPECT_EQ(LogFileUtil::AddPathSuffix("test.wpilog", "_sim"), "test_sim.wpilog");
        EXPECT_EQ(LogFileUtil::AddPathSuffix("logs/test.wpilog", "_sim"), "logs/test_sim.wpilog");
    }

    TEST(LogFileUtilTest, AddPathSuffixIndexesRepeatedSuffixes) {
        EXPECT_EQ(LogFileUtil::AddPathSuffix("test_sim.wpilog", "_sim"), "test_sim_2.wpilog");
        EXPECT_EQ(LogFileUtil::AddPathSuffix("test_sim_2.wpilog", "_sim"), "test_sim_3.wpilog");
        EXPECT_EQ(LogFileUtil::AddPathSuffix("test_sim_9.wpilog", "_sim"), "test_sim_10.wpilog");
    }

    TEST(LogFileUtilTest, AddPathSuffixLeavesPathsWithoutExtensionUnchanged) {
        EXPECT_EQ(LogFileUtil::AddPathSuffix("test", "_sim"), "test");
    }

    TEST_F(LogFileUtilEnvironmentTest, FindReplayLogEnvVarReadsEnvironmentVariable) {
        SetEnvironmentVariable("/tmp/replay.wpilog");
        EXPECT_EQ(LogFileUtil::FindReplayLogEnvVar(), std::optional<std::string>("/tmp/replay.wpilog"));
    }

    TEST_F(LogFileUtilEnvironmentTest, FindReplayLogEnvVarTreatsUnsetAsMissing) {
        UnsetEnvironmentVariable();
        EXPECT_EQ(LogFileUtil::FindReplayLogEnvVar(), std::nullopt);
    }

    TEST_F(LogFileUtilEnvironmentTest, FindReplayLogPrefersEnvironmentVariable) {
        SetEnvironmentVariable("/tmp/replay.wpilog");
        EXPECT_EQ(LogFileUtil::FindReplayLog(), "/tmp/replay.wpilog");
    }
} // namespace
