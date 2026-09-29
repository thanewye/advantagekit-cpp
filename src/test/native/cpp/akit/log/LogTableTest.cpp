#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include <frc/geometry/Rotation2d.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpi/struct/Struct.h>

#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/log/LogDataReceiver.h"
#include "akit/log/LogTable.h"

enum class TestMode { kDisabled, kAuto, kTeleop };

struct AggregateInputs {
    int64_t count;
    bool enabled;
    double position;
};

namespace {

    using akit::LogDataReceiver;
    using akit::Logger;
    using akit::LogStorage;
    using akit::LogTable;
    using akit::LogValue;
    using namespace units::literals;

    // PutTable now runs on the receiver thread while tests read from the main thread, so all
    // access to snapshots_ goes through mutex_; WaitForSnapshots lets a test block until async
    // dispatch has caught up instead of racing it.
    class CaptureReceiver : public LogDataReceiver {
    public:
        void PutTable(const LogTable& table) override {
            LogStorage snapshot;
            snapshot.values = table.GetAll();
            snapshot.timestamp = table.GetTimestamp();
            std::lock_guard lock(mutex_);
            snapshots_.push_back(std::move(snapshot));
            cv_.notify_all();
        }

        void Reset() {
            std::lock_guard lock(mutex_);
            snapshots_.clear();
        }

        bool WaitForSnapshots(size_t count, std::chrono::milliseconds timeout) {
            std::unique_lock lock(mutex_);
            return cv_.wait_for(lock, timeout, [this, count] { return snapshots_.size() >= count; });
        }

        [[nodiscard]] size_t SnapshotCount() {
            std::lock_guard lock(mutex_);
            return snapshots_.size();
        }

        [[nodiscard]] LogStorage SnapshotAt(size_t index) {
            std::lock_guard lock(mutex_);
            return snapshots_.at(index);
        }

    private:
        std::vector<LogStorage> snapshots_;
        std::mutex mutex_;
        std::condition_variable cv_;
    };

    CaptureReceiver g_captureReceiver;

    class ValidationLoggedRobot : public akit::LoggedRobot {
    public:
        ValidationLoggedRobot()
            : LoggedRobot() {}
    };

    void EnsureLoggedRobotValidationSatisfied() {
        static ValidationLoggedRobot robot;
    }

    std::vector<uint8_t> PackRotation(const frc::Rotation2d& rotation) {
        std::vector<uint8_t> bytes(wpi::Struct<frc::Rotation2d>::GetSize());
        wpi::PackStruct(std::span{bytes}, rotation);
        return bytes;
    }

    std::vector<uint8_t> PackRotations(const std::vector<frc::Rotation2d>& rotations) {
        const size_t elemSize = wpi::Struct<frc::Rotation2d>::GetSize();
        std::vector<uint8_t> bytes(elemSize * rotations.size());
        for (size_t i = 0; i < rotations.size(); ++i) {
            wpi::PackStruct(std::span{bytes}.subspan(i * elemSize, elemSize), rotations[i]);
        }
        return bytes;
    }

    TEST(LogTableStructTest, StructRoundTripSucceedsWhenTypeMatches) {
        LogStorage storage;
        LogTable table(storage);
        frc::Rotation2d expected{units::radian_t{1.234}};

        table.Put("rotation", expected);

        frc::Rotation2d actual = table.Get("rotation", frc::Rotation2d{});

        EXPECT_DOUBLE_EQ(actual.Radians().value(), expected.Radians().value());
    }

    TEST(LogTableStructTest, SingleStructReplayReturnsDefaultOnWrongCustomType) {
        LogStorage storage;
        LogTable table(storage);
        frc::Rotation2d expectedDefault{units::radian_t{0.75}};

        table.Put("rotation", LogValue{PackRotation(frc::Rotation2d{units::radian_t{1.234}}), "wrong:type"});

        frc::Rotation2d actual = table.Get("rotation", expectedDefault);

        EXPECT_DOUBLE_EQ(actual.Radians().value(), expectedDefault.Radians().value());
    }

    TEST(LogTableStructTest, SingleStructReplayReturnsDefaultOnWrongRawSize) {
        LogStorage storage;
        LogTable table(storage);
        frc::Rotation2d expectedDefault{units::radian_t{0.75}};
        std::vector<uint8_t> bytes(wpi::Struct<frc::Rotation2d>::GetSize() + 1, 0);

        table.Put("rotation", LogValue{std::move(bytes), std::string(wpi::GetStructTypeString<frc::Rotation2d>())});

        frc::Rotation2d actual = table.Get("rotation", expectedDefault);

        EXPECT_DOUBLE_EQ(actual.Radians().value(), expectedDefault.Radians().value());
    }

    TEST(LogTableStructTest, StructArrayReplayReturnsDefaultOnSingleStructTypeString) {
        LogStorage storage;
        LogTable table(storage);
        std::vector<frc::Rotation2d> expectedDefault{
            frc::Rotation2d{units::radian_t{0.25}},
            frc::Rotation2d{units::radian_t{0.5}},
        };
        std::vector<frc::Rotation2d> values{
            frc::Rotation2d{units::radian_t{1.0}},
            frc::Rotation2d{units::radian_t{2.0}},
        };

        table.Put("rotations", LogValue{PackRotations(values), std::string(wpi::GetStructTypeString<frc::Rotation2d>())});

        std::vector<frc::Rotation2d> actual = table.Get("rotations", expectedDefault);

        ASSERT_EQ(actual.size(), expectedDefault.size());
        for (size_t i = 0; i < actual.size(); ++i) {
            EXPECT_DOUBLE_EQ(actual[i].Radians().value(), expectedDefault[i].Radians().value());
        }
    }

    TEST(LogTableStructTest, StructArrayReplayReturnsDefaultOnNonDivisibleRawSize) {
        LogStorage storage;
        LogTable table(storage);
        std::vector<frc::Rotation2d> expectedDefault{frc::Rotation2d{units::radian_t{0.25}}};
        std::vector<uint8_t> bytes(wpi::Struct<frc::Rotation2d>::GetSize() + 1, 0);

        table.Put("rotations", LogValue{std::move(bytes), std::string(wpi::GetStructTypeString<frc::Rotation2d>()) + "[]"});

        std::vector<frc::Rotation2d> actual = table.Get("rotations", expectedDefault);

        ASSERT_EQ(actual.size(), expectedDefault.size());
        EXPECT_DOUBLE_EQ(actual[0].Radians().value(), expectedDefault[0].Radians().value());
    }

    TEST(LogTableWriteAllowedTest, TypedRawRejectsUntypedOverwrite) {
        LogStorage storage;
        LogTable table(storage);
        std::vector<uint8_t> original{1, 2, 3};
        std::vector<uint8_t> replacement{4, 5, 6};

        table.Put("raw", LogValue{original, "struct:test"});
        table.Put("raw", std::span<const uint8_t>{replacement});

        const LogValue* actual = table.Get("raw");
        ASSERT_NE(actual, nullptr);
        EXPECT_EQ(actual->customTypeStr, "struct:test");
        EXPECT_EQ(std::get<std::vector<uint8_t>>(actual->value), original);
    }

    TEST(LogTableWriteAllowedTest, UntypedRawRejectsTypedOverwrite) {
        LogStorage storage;
        LogTable table(storage);
        std::vector<uint8_t> original{1, 2, 3};
        std::vector<uint8_t> replacement{4, 5, 6};

        table.Put("raw", std::span<const uint8_t>{original});
        table.Put("raw", LogValue{replacement, "struct:test"});

        const LogValue* actual = table.Get("raw");
        ASSERT_NE(actual, nullptr);
        EXPECT_TRUE(actual->customTypeStr.empty());
        EXPECT_EQ(std::get<std::vector<uint8_t>>(actual->value), original);
    }

    TEST(LogTableWriteAllowedTest, SameKeyRewriteWithIdenticalMetadataSucceeds) {
        LogStorage storage;
        LogTable table(storage);
        std::vector<uint8_t> original{1, 2, 3};
        std::vector<uint8_t> replacement{4, 5, 6};

        table.Put("raw", LogValue{original, "struct:test"});
        table.Put("raw", LogValue{replacement, "struct:test"});

        const LogValue* actual = table.Get("raw");
        ASSERT_NE(actual, nullptr);
        EXPECT_EQ(actual->customTypeStr, "struct:test");
        EXPECT_EQ(std::get<std::vector<uint8_t>>(actual->value), replacement);
    }

    TEST(LogTableArrayParityTest, BooleanArrayRoundTripUsesBooleanArrayStorage) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<bool> expected{true, false, true};

        table.Put("flags", expected);

        const LogValue* stored = table.Get("flags");
        ASSERT_NE(stored, nullptr);
        EXPECT_EQ(stored->type, akit::LoggableType::kBooleanArray);
        EXPECT_EQ(std::get<std::vector<bool>>(stored->value), expected);
        EXPECT_EQ(table.Get("flags", expected), expected);
    }

    TEST(LogTableArrayParityTest, IntegerArrayRoundTripUsesIntegerArrayStorage) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<int> expected{1, -2, 3};

        table.Put("values", std::span<const int>(expected));

        const LogValue* stored = table.Get("values");
        ASSERT_NE(stored, nullptr);
        EXPECT_EQ(stored->type, akit::LoggableType::kIntegerArray);
        EXPECT_EQ(std::get<std::vector<int64_t>>(stored->value), (std::vector<int64_t>{1, -2, 3}));
        EXPECT_EQ(table.Get("values", std::span<const int>(expected)), expected);
    }

    TEST(LogTableArrayParityTest, RejectsRewritingBooleanArrayAsIntegerArray) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<bool> original{true, false, true};
        const std::vector<int> replacement{1, 2, 3};

        table.Put("array", original);
        table.Put("array", std::span<const int>(replacement));

        const LogValue* stored = table.Get("array");
        ASSERT_NE(stored, nullptr);
        EXPECT_EQ(stored->type, akit::LoggableType::kBooleanArray);
        EXPECT_EQ(std::get<std::vector<bool>>(stored->value), original);
    }

    TEST(LogTableArrayParityTest, RejectsRewritingIntegerArrayAsBooleanArray) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<int> original{1, 2, 3};
        const std::vector<bool> replacement{true, false, true};

        table.Put("array", std::span<const int>(original));
        table.Put("array", replacement);

        const LogValue* stored = table.Get("array");
        ASSERT_NE(stored, nullptr);
        EXPECT_EQ(stored->type, akit::LoggableType::kIntegerArray);
        EXPECT_EQ(std::get<std::vector<int64_t>>(stored->value), (std::vector<int64_t>{1, 2, 3}));
    }

    TEST(LogTableArrayParityTest, Integer2DArrayRoundTripUsesPerRowIntegerArrays) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<std::vector<int>> expected{{1, 2}, {3, 4, 5}};

        table.Put("values2d", std::span<const std::vector<int>>(expected));

        EXPECT_EQ(table.Get("values2d", std::span<const std::vector<int>>(expected)), expected);
    }

    TEST(LogTableArrayParityTest, Boolean2DArrayRoundTripUsesPerRowBooleanArrays) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<std::vector<bool>> expected{{true, false}, {false, true, true}};

        table.Put("flags2d", std::span<const std::vector<bool>>(expected));

        EXPECT_EQ(table.Get("flags2d", std::span<const std::vector<bool>>(expected)), expected);
    }

    TEST(LogTableStructTest, Struct2DArrayRoundTripUsesPerRowStructArrays) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<std::vector<frc::Rotation2d>> expected{
            {frc::Rotation2d{units::radian_t{0.5}}, frc::Rotation2d{units::radian_t{1.0}}},
            {frc::Rotation2d{units::radian_t{1.5}}},
        };

        table.Put("rotations2d", std::span<const std::vector<frc::Rotation2d>>(expected));

        const auto actual = table.Get("rotations2d", std::span<const std::vector<frc::Rotation2d>>(expected));
        ASSERT_EQ(actual.size(), expected.size());
        for (size_t row = 0; row < actual.size(); ++row) {
            ASSERT_EQ(actual[row].size(), expected[row].size());
            for (size_t col = 0; col < actual[row].size(); ++col) {
                EXPECT_DOUBLE_EQ(actual[row][col].Radians().value(), expected[row][col].Radians().value());
            }
        }
    }

    TEST(LogTableParityTest, RootKeysCanonicalizeLeadingSlashBehavior) {
        LogStorage storage;
        LogTable table(storage);

        table.Put("plain", 1);
        table.Put("/slash", 2);
        table.GetSubtable("/nested").Put("/value", 3);

        EXPECT_TRUE(storage.values.contains("/plain"));
        EXPECT_TRUE(storage.values.contains("/slash"));
        EXPECT_TRUE(storage.values.contains("/nested/value"));
        EXPECT_FALSE(storage.values.contains("plain"));
        EXPECT_FALSE(storage.values.contains("//slash"));
        EXPECT_FALSE(storage.values.contains("/nested//value"));
    }

    TEST(LogTableParityTest, AggregateSubtablePutGetRoundTrip) {
        LogStorage storage;
        LogTable table(storage);
        const AggregateInputs expected{7, true, 2.5};

        table.Put("aggregate", expected);

        const auto actual = table.Get("aggregate", AggregateInputs{});
        EXPECT_EQ(actual.count, expected.count);
        EXPECT_EQ(actual.enabled, expected.enabled);
        EXPECT_DOUBLE_EQ(actual.position, expected.position);
    }

    TEST(LogTableParityTest, Enum2DRoundTrip) {
        LogStorage storage;
        LogTable table(storage);
        const std::vector<std::vector<TestMode>> expected{{TestMode::kDisabled, TestMode::kAuto}, {TestMode::kTeleop}};

        table.Put("modes", std::span<const std::vector<TestMode>>(expected));

        EXPECT_EQ(table.Get("modes", std::span<const std::vector<TestMode>>(expected)), expected);
    }

    TEST(LogTableParityTest, MeasurePutUsesBaseUnitsAndMeasureGetRestoresCallerUnits) {
        LogStorage storage;
        LogTable table(storage);
        const auto expectedFeet = 1.0_ft;

        table.Put("distance", expectedFeet);

        const LogValue* stored = table.Get("distance");
        ASSERT_NE(stored, nullptr);
        ASSERT_TRUE(stored->unitStr.has_value());
        EXPECT_EQ(*stored->unitStr, units::meter_t{0}.name());
        EXPECT_NEAR(std::get<double>(stored->value), 0.3048, 1e-9);

        const auto actualFeet = table.Get("distance", 0.0_ft);
        EXPECT_NEAR(actualFeet.value(), expectedFeet.value(), 1e-9);
    }

    TEST(LoggerParityTest, PersistentSnapshotsRetainUnchangedValuesAcrossCycles) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();
        Logger::AddDataReceiver(&g_captureReceiver);
        g_captureReceiver.Reset();

        Logger::Start();
        g_captureReceiver.Reset();

        Logger::RecordOutput("Persisted", 1.0);
        Logger::RecordOutput("Changing", 10.0);
        Logger::PeriodicAfterUser();

        ASSERT_TRUE(g_captureReceiver.WaitForSnapshots(1, std::chrono::milliseconds(500)));
        ASSERT_EQ(g_captureReceiver.SnapshotCount(), 1u);
        EXPECT_TRUE(g_captureReceiver.SnapshotAt(0).values.contains("/RealOutputs/Persisted"));

        Logger::PeriodicBeforeUser();
        Logger::RecordOutput("Changing", 20.0);
        Logger::PeriodicAfterUser();

        ASSERT_TRUE(g_captureReceiver.WaitForSnapshots(2, std::chrono::milliseconds(500)));
        ASSERT_EQ(g_captureReceiver.SnapshotCount(), 2u);
        const auto secondSnapshot = g_captureReceiver.SnapshotAt(1).values;
        ASSERT_TRUE(secondSnapshot.contains("/RealOutputs/Persisted"));
        ASSERT_TRUE(secondSnapshot.contains("/RealOutputs/Changing"));
        EXPECT_DOUBLE_EQ(std::get<double>(secondSnapshot.at("/RealOutputs/Persisted").value), 1.0);
        EXPECT_DOUBLE_EQ(std::get<double>(secondSnapshot.at("/RealOutputs/Changing").value), 20.0);

        Logger::End();
        Logger::Clear();
        g_captureReceiver.Reset();
    }

    TEST(LoggerParityTest, EnqueuedSnapshotsAreIndependentOfCurrentStorageMutation) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();
        Logger::AddDataReceiver(&g_captureReceiver);
        g_captureReceiver.Reset();

        Logger::Start();
        g_captureReceiver.Reset();

        Logger::RecordOutput("Value", 1.0);
        Logger::PeriodicAfterUser(); // enqueues a clone with Value=1.0

        // Mutate currentStorage_ for the same key before the first snapshot is necessarily
        // dispatched. If Enqueue captured a reference instead of a deep copy, this would
        // retroactively change what the receiver eventually sees for the first snapshot.
        Logger::PeriodicBeforeUser();
        Logger::RecordOutput("Value", 2.0);
        Logger::PeriodicAfterUser();

        ASSERT_TRUE(g_captureReceiver.WaitForSnapshots(2, std::chrono::milliseconds(500)));
        const auto first = g_captureReceiver.SnapshotAt(0).values;
        ASSERT_TRUE(first.contains("/RealOutputs/Value"));
        EXPECT_DOUBLE_EQ(std::get<double>(first.at("/RealOutputs/Value").value), 1.0);

        const auto second = g_captureReceiver.SnapshotAt(1).values;
        ASSERT_TRUE(second.contains("/RealOutputs/Value"));
        EXPECT_DOUBLE_EQ(std::get<double>(second.at("/RealOutputs/Value").value), 2.0);

        Logger::End();
        Logger::Clear();
        g_captureReceiver.Reset();
    }

    TEST(LoggerParityTest, LateAddDataReceiverGetsSynchronousCatchUpSnapshot) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();
        Logger::Start();

        Logger::RecordOutput("Existing", 5.0);
        Logger::PeriodicAfterUser();

        CaptureReceiver lateReceiver;
        Logger::AddDataReceiver(&lateReceiver);

        // No waiting: the late-add catch-up call is synchronous on the calling thread.
        ASSERT_EQ(lateReceiver.SnapshotCount(), 1u);
        EXPECT_TRUE(lateReceiver.SnapshotAt(0).values.contains("/RealOutputs/Existing"));

        Logger::End();
        Logger::Clear();
        Logger::ClearReceivers(); // lateReceiver is about to go out of scope; must not leave a dangling pointer registered
    }

    TEST(LoggerParityTest, SupplierOverloadsForwardImmediatelyToValueOverloads) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();
        Logger::Start();

        Logger::RecordOutput("LambdaBool", [] { return true; });
        Logger::RecordOutput("LambdaInt", [] { return int{4}; });
        Logger::RecordOutput("LambdaLong", [] { return int64_t{9}; });
        Logger::RecordOutput("LambdaDouble", [] { return 1.25; });

        const auto& values = Logger::GetCurrentStorage().values;
        EXPECT_TRUE(std::get<bool>(values.at("/RealOutputs/LambdaBool").value));
        EXPECT_EQ(std::get<int64_t>(values.at("/RealOutputs/LambdaInt").value), 4);
        EXPECT_EQ(std::get<int64_t>(values.at("/RealOutputs/LambdaLong").value), 9);
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/RealOutputs/LambdaDouble").value), 1.25);

        Logger::End();
        Logger::Clear();
    }

    TEST(LoggerParityTest, ExtraConsoleDataAndTimingMetricsUseCanonicalOutputKeys) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();
        Logger::Start();

        Logger::PeriodicAfterUser(1'500, 500, "Exception line\nstack line\n");

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/Console"));
        EXPECT_EQ(std::get<std::string>(values.at("/RealOutputs/Console").value), "Exception line\nstack line");
        EXPECT_TRUE(values.contains("/RealOutputs/Logger/EntryUpdateMS"));
        EXPECT_TRUE(values.contains("/RealOutputs/Logger/AutoLogMS"));
        EXPECT_TRUE(values.contains("/RealOutputs/Logger/DriverStationMS"));
        EXPECT_TRUE(values.contains("/RealOutputs/Logger/ConsoleMS"));
        EXPECT_TRUE(values.contains("/RealOutputs/Logger/QueuedCycles"));
        EXPECT_TRUE(values.contains("/Logger/Timestamp"));
        EXPECT_TRUE(values.contains("/RealOutputs/LoggedRobot/FullCycleMS"));

        Logger::End();
        Logger::Clear();
    }

} // namespace
