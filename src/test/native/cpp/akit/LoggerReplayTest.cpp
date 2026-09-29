#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <hal/DriverStation.h>

#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/log/LogDataReceiver.h"
#include "akit/log/LogReplaySource.h"
#include "akit/log/LogStorage.h"
#include "akit/log/LogTable.h"
#include "akit/telemetry/LoggedDriverStation.h"

namespace {

    using akit::Logger;
    using akit::LogReplaySource;
    using akit::LogStorage;
    using akit::LogTable;

    class ValidationLoggedRobot : public akit::LoggedRobot {
    public:
        ValidationLoggedRobot()
            : LoggedRobot() {}
    };

    void EnsureLoggedRobotValidationSatisfied() {
        static ValidationLoggedRobot robot;
    }

    struct ReplayFrame {
        int64_t timestamp;
        std::function<void(LogTable&)> apply;
    };

    class StubReplaySource : public LogReplaySource {
    public:
        explicit StubReplaySource(std::vector<ReplayFrame> frames)
            : frames_(std::move(frames)) {}

        void Start() override { startCalls++; }
        void End() override { endCalls++; }

        bool UpdateTable(LogTable& table) override {
            if (nextFrame_ >= frames_.size()) {
                return false;
            }

            const auto& frame = frames_[nextFrame_++];
            table.SetTimestamp(frame.timestamp);
            if (frame.apply) {
                frame.apply(table);
            }
            return true;
        }

        int startCalls = 0;
        int endCalls = 0;

    private:
        std::vector<ReplayFrame> frames_;
        size_t nextFrame_ = 0;
    };

    class CaptureReceiver : public akit::LogDataReceiver {
    public:
        void End() override { endCalls++; }

        void PutTable(const LogTable& table) override {
            LogStorage snapshot;
            snapshot.values = table.GetAll();
            snapshot.timestamp = table.GetTimestamp();
            snapshots.push_back(std::move(snapshot));
        }

        std::vector<LogStorage> snapshots;
        int endCalls = 0;
    };

    void ResetLoggerState() {
        Logger::End();
        Logger::ClearReceivers();
        Logger::SetReplaySource(nullptr);
        Logger::Clear();
        frc::sim::DriverStationSim::ResetData();
        frc::DriverStation::RefreshData();
    }

    class LoggerReplayParityTest : public ::testing::Test {
    protected:
        void SetUp() override {
            if (!frc::RobotBase::IsSimulation()) {
                GTEST_SKIP();
            }
            EnsureLoggedRobotValidationSatisfied();
            ResetLoggerState();
        }

        void TearDown() override { ResetLoggerState(); }
    };

    TEST_F(LoggerReplayParityTest, ReplayStartupLoadsInitialFrameAndWritesReplayOutputs) {
        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) { table.GetSubtable("ReplayInputs").Put("Applied", 12.5); },
            },
        });

        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        EXPECT_TRUE(Logger::IsRunning());
        EXPECT_TRUE(Logger::HasReplaySource());
        EXPECT_EQ(replaySource.startCalls, 1);
        EXPECT_EQ(Logger::GetCurrentStorage().timestamp, 1'000);
        ASSERT_TRUE(Logger::GetCurrentStorage().values.contains("/ReplayInputs/Applied"));
        EXPECT_DOUBLE_EQ(std::get<double>(Logger::GetCurrentStorage().values.at("/ReplayInputs/Applied").value), 12.5);

        Logger::RecordOutput("ReplayValue", 7.5);

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/ReplayOutputs/ReplayValue"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/ReplayOutputs/ReplayValue").value), 7.5);

        // Must end while replaySource (a local) is still alive: Logger::replaySource_ would
        // otherwise dangle once this scope exits, and the fixture's TearDown() calls End() again.
        Logger::End();
    }

    TEST_F(LoggerReplayParityTest, ReplayBeforeUserUpdatesTimestampAndDriverStationState) {
        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) {
                    auto ds = table.GetSubtable("DriverStation");
                    ds.Put("Enabled", false);
                    ds.Put("Autonomous", true);
                    ds.Put("DSAttached", true);
                    ds.Put("FMSAttached", true);
                    ds.Put("MatchNumber", 4);
                    ds.Put("ReplayNumber", 1);
                    ds.Put("MatchType", static_cast<int64_t>(2));
                    ds.Put("EventName", "Week Zero");
                    ds.Put("MatchTime", 15.0);
                    ds.GetSubtable("Joystick0").Put("Name", "Replay Pad");
                    ds.GetSubtable("Joystick0").Put("Type", static_cast<int64_t>(frc::GenericHID::HIDType::kHIDGamepad));
                    ds.GetSubtable("Joystick0").Put("ButtonCount", 2);
                    ds.GetSubtable("Joystick0").Put("ButtonValues", static_cast<int64_t>(0b01));
                    std::vector<float> axisValues{0.25f};
                    ds.GetSubtable("Joystick0").Put("AxisValues", std::span<const float>(axisValues));
                    std::vector<int> axisTypes{static_cast<int>(frc::Joystick::AxisType::kYAxis)};
                    ds.GetSubtable("Joystick0").Put("AxisTypes", std::span<const int>(axisTypes));
                },
            },
            ReplayFrame{
                2'000,
                [](LogTable& table) {
                    auto ds = table.GetSubtable("DriverStation");
                    ds.Put("Enabled", true);
                    ds.Put("Autonomous", false);
                    ds.Put("Test", true);
                    ds.Put("DSAttached", true);
                    ds.Put("FMSAttached", false);
                    ds.Put("MatchNumber", 9);
                    ds.Put("ReplayNumber", 2);
                    ds.Put("MatchType", static_cast<int64_t>(3));
                    ds.Put("EventName", "Replay Event");
                    ds.Put("MatchTime", 9.5);
                    ds.GetSubtable("Joystick0").Put("Name", "Updated Pad");
                    ds.GetSubtable("Joystick0").Put("Type", static_cast<int64_t>(frc::GenericHID::HIDType::kHIDJoystick));
                    ds.GetSubtable("Joystick0").Put("ButtonCount", 3);
                    ds.GetSubtable("Joystick0").Put("ButtonValues", static_cast<int64_t>(0b101));
                    std::vector<float> axisValues{0.75f, -0.25f};
                    ds.GetSubtable("Joystick0").Put("AxisValues", std::span<const float>(axisValues));
                    std::vector<int> axisTypes{
                        static_cast<int>(frc::Joystick::AxisType::kZAxis),
                        static_cast<int>(frc::Joystick::AxisType::kTwistAxis),
                    };
                    ds.GetSubtable("Joystick0").Put("AxisTypes", std::span<const int>(axisTypes));
                },
            },
        });

        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        frc::DriverStation::RefreshData();
        EXPECT_FALSE(frc::DriverStation::IsEnabled());
        EXPECT_TRUE(frc::DriverStation::IsAutonomous());
        EXPECT_EQ(frc::DriverStation::GetMatchNumber(), 4);
        EXPECT_EQ(frc::DriverStation::GetReplayNumber(), 1);
        EXPECT_EQ(frc::DriverStation::GetEventName(), "Week Zero");
        EXPECT_EQ(frc::DriverStation::GetJoystickName(0), "Replay Pad");

        Logger::PeriodicBeforeUser();

        EXPECT_EQ(Logger::GetCurrentStorage().timestamp, 2'000);
        frc::DriverStation::RefreshData();
        EXPECT_TRUE(frc::DriverStation::IsEnabled());
        EXPECT_FALSE(frc::DriverStation::IsAutonomous());
        EXPECT_TRUE(frc::DriverStation::IsTest());
        EXPECT_EQ(frc::DriverStation::GetMatchNumber(), 9);
        EXPECT_EQ(frc::DriverStation::GetReplayNumber(), 2);
        EXPECT_EQ(frc::DriverStation::GetEventName(), "Replay Event");
        EXPECT_EQ(frc::DriverStation::GetJoystickName(0), "Updated Pad");
        EXPECT_NEAR(frc::DriverStation::GetMatchTime().value(), 9.5, 1e-9);

        // Must end while replaySource (a local) is still alive: Logger::replaySource_ would
        // otherwise dangle once this scope exits, and the fixture's TearDown() calls End() again.
        Logger::End();
    }

    TEST_F(LoggerReplayParityTest, ReplayExhaustionEndsLoggerExactlyOnce) {
        CaptureReceiver receiver;
        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) { table.GetSubtable("DriverStation").Put("DSAttached", true); },
            },
        });

        Logger::AddDataReceiver(&receiver);
        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        ASSERT_TRUE(Logger::IsRunning());
        Logger::PeriodicBeforeUser();

        EXPECT_FALSE(Logger::IsRunning());
        EXPECT_EQ(replaySource.endCalls, 1);
        EXPECT_EQ(receiver.endCalls, 1);

        Logger::RecordOutput("AfterEnd", 1.0);
        EXPECT_FALSE(Logger::GetCurrentStorage().values.contains("/ReplayOutputs/AfterEnd"));

        Logger::End();
        EXPECT_EQ(replaySource.endCalls, 1);
        EXPECT_EQ(receiver.endCalls, 1);
    }

    TEST_F(LoggerReplayParityTest, ReplayAfterUserPreservesReplayedSystemStats) {
        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) {
                    auto systemStats = table.GetSubtable("SystemStats");
                    systemStats.Put("BatteryVoltage", 12.34);
                    systemStats.Put("EpochTimeMicros", static_cast<int64_t>(123456789));
                    systemStats.GetSubtable("NTClients").GetSubtable("ReplayClient").Put("Connected", true);
                },
            },
        });

        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/SystemStats/BatteryVoltage"));
        ASSERT_TRUE(values.contains("/SystemStats/EpochTimeMicros"));
        ASSERT_TRUE(values.contains("/SystemStats/NTClients/ReplayClient/Connected"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/SystemStats/BatteryVoltage").value), 12.34);
        EXPECT_EQ(std::get<int64_t>(values.at("/SystemStats/EpochTimeMicros").value), 123456789);
        EXPECT_TRUE(std::get<bool>(values.at("/SystemStats/NTClients/ReplayClient/Connected").value));

        Logger::End();
    }

    TEST_F(LoggerReplayParityTest, RealModeStillUsesLiveTimestampAndRealOutputs) {
        Logger::Start();

        EXPECT_TRUE(Logger::IsRunning());
        EXPECT_FALSE(Logger::HasReplaySource());
        EXPECT_GT(Logger::GetCurrentStorage().timestamp, 0);

        const auto fpgaTimeUs = static_cast<int64_t>(frc::RobotController::GetFPGATime());
        EXPECT_LE(std::llabs(Logger::GetCurrentStorage().timestamp - fpgaTimeUs), 100'000);

        Logger::RecordOutput("RealValue", 3.5);
        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/RealValue"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/RealOutputs/RealValue").value), 3.5);
    }

    TEST_F(LoggerReplayParityTest, DriverStationSaveUsesIntegerEncodingShape) {
        frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue2);
        frc::sim::DriverStationSim::SetEventName("  Week Zero  ");
        frc::sim::DriverStationSim::SetGameSpecificMessage("  ABC  ");
        frc::sim::DriverStationSim::SetMatchNumber(4);
        frc::sim::DriverStationSim::SetReplayNumber(1);
        frc::sim::DriverStationSim::SetMatchType(frc::DriverStation::MatchType::kQualification);
        frc::sim::DriverStationSim::SetJoystickName(0, "  Replay Pad  ");
        frc::sim::DriverStationSim::SetJoystickType(0, 20);
        frc::sim::DriverStationSim::SetJoystickAxisCount(0, 2);
        frc::sim::DriverStationSim::SetJoystickAxisType(0, 0, 1);
        frc::sim::DriverStationSim::SetJoystickAxisType(0, 1, 2);
        frc::sim::DriverStationSim::NotifyNewData();
        frc::DriverStation::RefreshData();

        LogStorage storage;
        LogTable table(storage);
        akit::LoggedDriverStation::SaveToLog(table.GetSubtable("DriverStation"));

        const auto& values = storage.values;
        ASSERT_TRUE(values.contains("/DriverStation/AllianceStation"));
        ASSERT_TRUE(values.contains("/DriverStation/EventName"));
        ASSERT_TRUE(values.contains("/DriverStation/GameSpecificMessage"));
        ASSERT_TRUE(values.contains("/DriverStation/MatchType"));
        ASSERT_TRUE(values.contains("/DriverStation/Joystick0/Name"));
        ASSERT_TRUE(values.contains("/DriverStation/Joystick0/Type"));
        ASSERT_TRUE(values.contains("/DriverStation/Joystick0/AxisTypes"));
        EXPECT_EQ(values.at("/DriverStation/AllianceStation").type, akit::LoggableType::kInteger);
        EXPECT_EQ(values.at("/DriverStation/MatchType").type, akit::LoggableType::kInteger);
        EXPECT_EQ(values.at("/DriverStation/Joystick0/Type").type, akit::LoggableType::kInteger);
        EXPECT_EQ(values.at("/DriverStation/Joystick0/AxisTypes").type, akit::LoggableType::kIntegerArray);
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/AllianceStation").value), HAL_AllianceStationID_kBlue2);
        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/EventName").value), "Week Zero");
        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/GameSpecificMessage").value), "ABC");
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/MatchType").value), 2);
        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/Joystick0/Name").value), "Replay Pad");
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/Type").value), 20);
        EXPECT_EQ(std::get<std::vector<int64_t>>(values.at("/DriverStation/Joystick0/AxisTypes").value), (std::vector<int64_t>{1, 2}));
    }

} // namespace
