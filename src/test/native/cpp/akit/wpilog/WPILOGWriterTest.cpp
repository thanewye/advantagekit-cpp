#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>

#include "akit/log/LogStorage.h"
#include "akit/log/LogTable.h"
#include "akit/wpilog/WPILOGWriter.h"

namespace {
    using akit::LogStorage;
    using akit::LogTable;
    using akit::LogValue;
    using akit::wpilog::WPILOGWriter;

    struct LogCycle {
        int64_t timestamp;
        std::vector<std::pair<std::string, LogValue>> values;
    };

    class WPILOGWriterTest : public ::testing::Test {
    protected:
        void SetUp() override {
            logPath_ = (std::filesystem::temp_directory_path() /
                        (std::string("akit_writer_") + ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".wpilog"))
                           .string();
        }

        void TearDown() override {
            std::error_code ec;
            std::filesystem::remove(logPath_, ec);
        }

        void WriteLog(const std::vector<LogCycle>& cycles) const {
            WPILOGWriter writer(logPath_, WPILOGWriter::AdvantageScopeOpenBehavior::kNever);
            writer.Start();
            for (const auto& cycle : cycles) {
                LogStorage storage;
                LogTable table(storage);
                table.SetTimestamp(cycle.timestamp);
                for (const auto& [key, value] : cycle.values)
                    table.Put(key, value);
                writer.PutTable(table);
            }
            writer.End();
        }

        [[nodiscard]] std::map<std::string, int> CountDataRecordsByKey() const {
            auto buffer = wpi::MemoryBuffer::GetFile(logPath_);
            EXPECT_TRUE(buffer.has_value());
            wpi::log::DataLogReader reader(std::move(*buffer));
            std::map<int, std::string> keysByEntry;
            std::map<std::string, int> recordCounts;
            for (const auto& record : reader) {
                if (record.IsStart()) {
                    wpi::log::StartRecordData startData;
                    if (record.GetStartData(&startData)) keysByEntry[startData.entry] = std::string(startData.name);
                } else if (!record.IsControl()) {
                    ++recordCounts[keysByEntry[record.GetEntry()]];
                }
            }
            return recordCounts;
        }

        std::string logPath_;
    };

    TEST_F(WPILOGWriterTest, UnchangedNaNIsWrittenOnce) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        WriteLog({
            LogCycle{1'000, {{"Value", LogValue{nan}}}},
            LogCycle{2'000, {{"Value", LogValue{nan}}}},
            LogCycle{3'000, {{"Value", LogValue{nan}}}},
        });

        EXPECT_EQ(CountDataRecordsByKey()["/Value"], 1);
    }

    TEST_F(WPILOGWriterTest, UnchangedNaNArrayIsWrittenOnce) {
        const std::vector<double> withNaN{1.0, std::numeric_limits<double>::quiet_NaN()};
        WriteLog({
            LogCycle{1'000, {{"Values", LogValue{withNaN}}}},
            LogCycle{2'000, {{"Values", LogValue{withNaN}}}},
        });

        EXPECT_EQ(CountDataRecordsByKey()["/Values"], 1);
    }

    TEST_F(WPILOGWriterTest, SignFlipOfZeroIsWritten) {
        WriteLog({
            LogCycle{1'000, {{"Value", LogValue{0.0}}}},
            LogCycle{2'000, {{"Value", LogValue{-0.0}}}},
        });

        EXPECT_EQ(CountDataRecordsByKey()["/Value"], 2);
    }

    TEST_F(WPILOGWriterTest, UnchangedValueIsRewrittenAfterMissingForACycle) {
        WriteLog({
            LogCycle{1'000, {{"Value", LogValue{1.0}}}},
            LogCycle{2'000, {{"Value", LogValue{1.0}}}},
            LogCycle{3'000, {{"Other", LogValue{2.0}}}},
            LogCycle{4'000, {{"Value", LogValue{1.0}}}},
        });

        EXPECT_EQ(CountDataRecordsByKey()["/Value"], 2);
    }

    TEST_F(WPILOGWriterTest, ChangedValuesAreWrittenEveryCycle) {
        WriteLog({
            LogCycle{1'000, {{"Value", LogValue{int64_t{1}}}}},
            LogCycle{2'000, {{"Value", LogValue{int64_t{2}}}}},
            LogCycle{3'000, {{"Value", LogValue{int64_t{2}}}}},
            LogCycle{4'000, {{"Value", LogValue{int64_t{3}}}}},
        });

        EXPECT_EQ(CountDataRecordsByKey()["/Value"], 3);
    }
} // namespace
