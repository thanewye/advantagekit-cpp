#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/datalog/DataLogWriter.hpp>

#include "akit/log/LogStorage.h"
#include "akit/log/LogTable.h"
#include "akit/wpilog/WPILOGConstants.h"
#include "akit/wpilog/WPILOGReader.h"
#include "akit/wpilog/WPILOGWriter.h"

namespace {
    using akit::LogStorage;
    using akit::LogTable;
    using akit::LogValue;
    using akit::wpilog::WPILOGReader;
    using akit::wpilog::WPILOGWriter;

    std::string MetadataWithUnit(std::string_view unit) {
        std::string metadata{akit::WPILOGConstants::kEntryMetadataUnits};
        metadata.replace(metadata.find("$UNITSTR"), 8, unit);
        return metadata;
    }

    struct LogCycle {
        int64_t timestamp;
        std::vector<std::pair<std::string, LogValue>> values;
    };

    class WPILOGReaderTest : public ::testing::Test {
    protected:
        void SetUp() override {
            logPath_ = (std::filesystem::temp_directory_path() /
                        (std::string("akit_reader_") + ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".wpilog"))
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

        std::string logPath_;
    };

    TEST_F(WPILOGReaderTest, RoundTripsEveryValueType) {
        WriteLog({
            LogCycle{1'000,
                     {
                         {"Raw", LogValue{std::vector<uint8_t>{1, 2, 3}}},
                         {"Boolean", LogValue{true}},
                         {"Integer", LogValue{int64_t{42}}},
                         {"Float", LogValue{1.5f}},
                         {"Double", LogValue{2.25}},
                         {"String", LogValue{std::string("hello")}},
                         {"BooleanArray", LogValue{std::vector<bool>{true, false, true}}},
                         {"IntegerArray", LogValue{std::vector<int64_t>{1, -2, 3}}},
                         {"FloatArray", LogValue{std::vector<float>{0.5f, -0.25f}}},
                         {"DoubleArray", LogValue{std::vector<double>{1.0, 2.0}}},
                         {"StringArray", LogValue{std::vector<std::string>{"a", "b"}}},
                         {"Struct", LogValue{std::vector<uint8_t>{4, 5}, "struct:Pose2d"}},
                         {"Json", LogValue{std::string("{}"), "json"}},
                         {"Distance", LogValue{3.0, "", "meters"}},
                         {"Angle", LogValue{0.5f, "", "radians"}},
                     }},
            LogCycle{2'000, {{"Final", LogValue{true}}}},
        });

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.timestamp, 1'000);

        const auto& values = storage.values;
        EXPECT_EQ(values.at("/Raw"), LogValue(std::vector<uint8_t>{1, 2, 3}));
        EXPECT_EQ(values.at("/Boolean"), LogValue(true));
        EXPECT_EQ(values.at("/Integer"), LogValue(int64_t{42}));
        EXPECT_EQ(values.at("/Float"), LogValue(1.5f));
        EXPECT_EQ(values.at("/Double"), LogValue(2.25));
        EXPECT_EQ(values.at("/String"), LogValue(std::string("hello")));
        EXPECT_EQ(values.at("/BooleanArray"), LogValue(std::vector<bool>{true, false, true}));
        EXPECT_EQ(values.at("/IntegerArray"), LogValue(std::vector<int64_t>{1, -2, 3}));
        EXPECT_EQ(values.at("/FloatArray"), LogValue(std::vector<float>{0.5f, -0.25f}));
        EXPECT_EQ(values.at("/DoubleArray"), LogValue(std::vector<double>{1.0, 2.0}));
        EXPECT_EQ(values.at("/StringArray"), LogValue(std::vector<std::string>{"a", "b"}));
        EXPECT_EQ(values.at("/Struct"), LogValue(std::vector<uint8_t>{4, 5}, "struct:Pose2d"));
        EXPECT_EQ(values.at("/Json"), LogValue(std::string("{}"), "json"));
        EXPECT_EQ(values.at("/Distance"), LogValue(3.0, "", "meters"));
        EXPECT_EQ(values.at("/Angle"), LogValue(0.5f, "", "radians"));
    }

    TEST_F(WPILOGReaderTest, AdvancesOneCycleAtATimeAndKeepsUnchangedValues) {
        WriteLog({
            LogCycle{1'000, {{"Constant", LogValue{1.0}}, {"Changing", LogValue{int64_t{1}}}}},
            LogCycle{2'000, {{"Constant", LogValue{1.0}}, {"Changing", LogValue{int64_t{2}}}}},
            LogCycle{3'000, {{"Constant", LogValue{1.0}}, {"Changing", LogValue{int64_t{3}}}}},
        });

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.timestamp, 1'000);
        EXPECT_EQ(storage.values.at("/Changing"), LogValue(int64_t{1}));

        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.timestamp, 2'000);
        EXPECT_EQ(storage.values.at("/Changing"), LogValue(int64_t{2}));
        EXPECT_EQ(storage.values.at("/Constant"), LogValue(1.0));

        EXPECT_FALSE(reader.UpdateTable(table));
        EXPECT_EQ(storage.timestamp, 3'000);
        EXPECT_EQ(storage.values.at("/Changing"), LogValue(int64_t{3}));
    }

    TEST_F(WPILOGReaderTest, TracksUnitChangesFromMetadataRecords) {
        WriteLog({
            LogCycle{1'000, {{"Distance", LogValue{1.0, "", "meters"}}}},
            LogCycle{2'000, {{"Distance", LogValue{2.0, "", "inches"}}}},
            LogCycle{3'000, {{"Distance", LogValue{3.0}}}},
            LogCycle{4'000, {{"Final", LogValue{true}}}},
        });

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.values.at("/Distance"), LogValue(1.0, "", "meters"));
        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.values.at("/Distance"), LogValue(2.0, "", "inches"));
        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.values.at("/Distance"), LogValue(3.0));
    }

    TEST_F(WPILOGReaderTest, SkipsPreviousReplayOutputs) {
        WriteLog({
            LogCycle{1'000, {{"ReplayOutputs/Old", LogValue{1.0}}, {"RealOutputs/Kept", LogValue{2.0}}}},
            LogCycle{2'000, {{"Final", LogValue{true}}}},
        });

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_FALSE(storage.values.contains("/ReplayOutputs/Old"));
        EXPECT_EQ(storage.values.at("/RealOutputs/Kept"), LogValue(2.0));
    }

    TEST_F(WPILOGReaderTest, MissingFileReturnsFalse) {
        WPILOGReader reader(logPath_ + ".missing");
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        EXPECT_FALSE(reader.UpdateTable(table));
    }

    TEST_F(WPILOGReaderTest, LogWithoutAdvantageKitHeaderReturnsFalse) {
        {
            std::error_code ec;
            wpi::log::DataLogWriter log(logPath_, ec, "");
            ASSERT_FALSE(ec);
            const int entry = log.Start("/Timestamp", "int64", "", 0);
            log.AppendInteger(entry, 1'000, 1'000);
        }

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        EXPECT_FALSE(reader.UpdateTable(table));
    }

    TEST_F(WPILOGReaderTest, LogWithMicrosecondTimestampsIsRejected) {
        {
            std::error_code ec;
            wpi::log::DataLogWriter log(logPath_, ec, akit::WPILOGConstants::kExtraHeader);
            ASSERT_FALSE(ec);
            const int entry = log.Start("/Timestamp", "int64", akit::WPILOGConstants::kEntryMetadata, 0);
            log.AppendInteger(entry, 1'000, 1'000);
        }

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        EXPECT_THROW(reader.UpdateTable(table), std::runtime_error);
    }

    TEST_F(WPILOGReaderTest, ValuesWithRecordTimestampsOffTheCycleTimestampAreStillApplied) {
        {
            std::error_code ec;
            wpi::log::DataLogWriter log(logPath_, ec, akit::WPILOGConstants::kExtraHeader);
            ASSERT_FALSE(ec);
            const int timestampEntry = log.Start("/Timestamp", "int64", MetadataWithUnit("nanoseconds"), 0);
            const int valueEntry = log.Start("/Inputs/Value", "double", akit::WPILOGConstants::kEntryMetadata, 0);
            log.AppendInteger(timestampEntry, 1'000'000, 1'000'000);
            log.AppendDouble(valueEntry, 1.5, 1'000'250);
            log.AppendInteger(timestampEntry, 21'000'000, 21'000'000);
            log.AppendDouble(valueEntry, 2.5, 21'000'000);
            log.AppendInteger(timestampEntry, 41'000'000, 41'000'000);
        }

        WPILOGReader reader(logPath_);
        reader.Start();
        LogStorage storage;
        LogTable table(storage);

        ASSERT_TRUE(reader.UpdateTable(table));
        EXPECT_EQ(storage.timestamp, 1'000'000);
        EXPECT_EQ(storage.values.at("/Inputs/Value"), LogValue(1.5));

        reader.UpdateTable(table);
        EXPECT_EQ(storage.timestamp, 21'000'000);
        EXPECT_EQ(storage.values.at("/Inputs/Value"), LogValue(2.5));
    }
} // namespace
