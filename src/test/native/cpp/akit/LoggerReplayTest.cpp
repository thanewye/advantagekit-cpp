#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/driverstation/MatchType.hpp>
#include <wpi/driverstation/POVDirection.hpp>
#include <wpi/driverstation/internal/DriverStationBackend.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/hal/DriverStation.h>
#include <wpi/hal/DriverStationTypes.hpp>
#include <wpi/hal/simulation/DriverStationData.h>
#include <wpi/simulation/DriverStationSim.hpp>
#include <wpi/system/RobotController.hpp>

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
    using wpi::internal::DriverStationBackend;

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
        wpi::sim::DriverStationSim::ResetData();
        DriverStationBackend::RefreshData();
    }

    class LoggerReplayParityTest : public ::testing::Test {
    protected:
        void SetUp() override {
            if (!wpi::RobotBase::IsSimulation()) {
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
        static constexpr int64_t kUtilityOpModeId = HAL_MAKE_OPMODEID(HAL_ROBOT_MODE_UTILITY, 0x1234);

        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) {
                    auto ds = table.GetSubtable("DriverStation");
                    ds.Put("Enabled", false);
                    ds.Put("RobotMode", wpi::hal::RobotMode::AUTONOMOUS);
                    ds.Put("DSAttached", true);
                    ds.Put("FMSAttached", true);
                    ds.Put("MatchNumber", 4);
                    ds.Put("ReplayNumber", 1);
                    ds.Put("MatchType", static_cast<int64_t>(2));
                    ds.Put("EventName", "Week Zero");
                    ds.Put("GameData", "ABC");
                    ds.Put("MatchTime", 15.0);
                    auto joystick = ds.GetSubtable("Joystick0");
                    joystick.Put("Name", "Replay Pad");
                    joystick.Put("Type", static_cast<int64_t>(1));
                    joystick.Put("IsGamepad", true);
                    joystick.Put("ButtonsAvailable", static_cast<int64_t>(0b11));
                    joystick.Put("ButtonValues", static_cast<int64_t>(0b01));
                    joystick.Put("AxesAvailable", static_cast<int64_t>(0b1));
                    std::vector<float> axisValues{0.25f};
                    joystick.Put("AxisValues", std::span<const float>(axisValues));
                    joystick.Put("POVsAvailable", static_cast<int64_t>(0b1));
                    std::vector<int> povValues{HAL_JOYSTICK_POV_UP};
                    joystick.Put("POVValues", std::span<const int>(povValues));
                    joystick.Put("TouchpadCount", static_cast<int64_t>(1));
                    joystick.Put("Touchpad/0/FingerCount", static_cast<int64_t>(1));
                    joystick.Put("Touchpad/0/Finger/0/Down", true);
                    joystick.Put("Touchpad/0/Finger/0/X", 0.5f);
                    joystick.Put("Touchpad/0/Finger/0/Y", 0.25f);
                },
            },
            ReplayFrame{
                2'000,
                [](LogTable& table) {
                    auto ds = table.GetSubtable("DriverStation");
                    ds.Put("Enabled", true);
                    ds.Put("RobotMode", wpi::hal::RobotMode::UTILITY);
                    ds.Put("OpModeId", kUtilityOpModeId);
                    ds.Put("DSAttached", true);
                    ds.Put("FMSAttached", false);
                    ds.Put("MatchNumber", 9);
                    ds.Put("ReplayNumber", 2);
                    ds.Put("MatchType", static_cast<int64_t>(3));
                    ds.Put("EventName", "Replay Event");
                    ds.Put("MatchTime", 9.5);
                    auto joystick = ds.GetSubtable("Joystick0");
                    joystick.Put("Name", "Updated Pad");
                    joystick.Put("Type", static_cast<int64_t>(2));
                    joystick.Put("IsGamepad", false);
                    joystick.Put("ButtonsAvailable", static_cast<int64_t>(0b111));
                    joystick.Put("ButtonValues", static_cast<int64_t>(0b101));
                    joystick.Put("AxesAvailable", static_cast<int64_t>(0b11));
                    std::vector<float> axisValues{0.75f, -0.25f};
                    joystick.Put("AxisValues", std::span<const float>(axisValues));
                },
            },
        });

        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        DriverStationBackend::RefreshData();
        EXPECT_FALSE(DriverStationBackend::IsEnabled());
        EXPECT_TRUE(DriverStationBackend::IsAutonomous());
        EXPECT_TRUE(DriverStationBackend::IsFMSAttached());
        EXPECT_EQ(DriverStationBackend::GetMatchNumber(), 4);
        EXPECT_EQ(DriverStationBackend::GetReplayNumber(), 1);
        EXPECT_EQ(DriverStationBackend::GetMatchType(), wpi::MatchType::QUALIFICATION);
        EXPECT_EQ(DriverStationBackend::GetEventName(), "Week Zero");
        EXPECT_EQ(DriverStationBackend::GetGameData(), std::optional<std::string>{"ABC"});
        EXPECT_EQ(DriverStationBackend::GetJoystickName(0), "Replay Pad");
        EXPECT_TRUE(DriverStationBackend::GetJoystickIsGamepad(0));
        EXPECT_EQ(DriverStationBackend::GetStickButtonsAvailable(0), 0b11u);
        EXPECT_EQ(DriverStationBackend::GetStickButtons(0), 0b01u);
        EXPECT_NEAR(DriverStationBackend::GetStickAxis(0, 0), 0.25, 1e-6);
        EXPECT_EQ(DriverStationBackend::GetStickPOV(0, 0), wpi::POVDirection::UP);
        const auto finger = DriverStationBackend::GetStickTouchpadFinger(0, 0, 0);
        EXPECT_TRUE(finger.down);
        EXPECT_NEAR(finger.x, 0.5f, 1e-6);
        EXPECT_NEAR(finger.y, 0.25f, 1e-6);

        Logger::PeriodicBeforeUser();

        EXPECT_EQ(Logger::GetCurrentStorage().timestamp, 2'000);
        DriverStationBackend::RefreshData();
        EXPECT_TRUE(DriverStationBackend::IsEnabled());
        EXPECT_FALSE(DriverStationBackend::IsAutonomous());
        EXPECT_TRUE(DriverStationBackend::IsUtility());
        EXPECT_EQ(DriverStationBackend::GetControlWord().GetOpModeId(), kUtilityOpModeId);
        EXPECT_EQ(DriverStationBackend::GetMatchNumber(), 9);
        EXPECT_EQ(DriverStationBackend::GetReplayNumber(), 2);
        EXPECT_EQ(DriverStationBackend::GetEventName(), "Replay Event");
        EXPECT_EQ(DriverStationBackend::GetJoystickName(0), "Updated Pad");
        EXPECT_FALSE(DriverStationBackend::GetJoystickIsGamepad(0));
        EXPECT_EQ(DriverStationBackend::GetStickButtons(0), 0b101u);
        EXPECT_NEAR(DriverStationBackend::GetStickAxis(0, 1), -0.25, 1e-6);
        EXPECT_NEAR(DriverStationBackend::GetMatchTime().value(), 9.5, 1e-9);

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

    TEST_F(LoggerReplayParityTest, ReplayDispatchesEachCycleSynchronouslyWithoutQueueing) {
        CaptureReceiver receiver;
        StubReplaySource replaySource({
            ReplayFrame{1'000, [](LogTable& table) { table.GetSubtable("ReplayInputs").Put("Applied", 1.0); }},
            ReplayFrame{2'000, [](LogTable& table) { table.GetSubtable("ReplayInputs").Put("Applied", 2.0); }},
            ReplayFrame{3'000, {}},
        });

        Logger::AddDataReceiver(&receiver);
        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        Logger::PeriodicAfterUser();
        ASSERT_EQ(receiver.snapshots.size(), 1u);
        EXPECT_EQ(receiver.snapshots[0].timestamp, 1'000);
        EXPECT_DOUBLE_EQ(std::get<double>(receiver.snapshots[0].values.at("/ReplayInputs/Applied").value), 1.0);

        Logger::PeriodicBeforeUser();
        Logger::PeriodicAfterUser();
        ASSERT_EQ(receiver.snapshots.size(), 2u);
        EXPECT_EQ(receiver.snapshots[1].timestamp, 2'000);
        EXPECT_DOUBLE_EQ(std::get<double>(receiver.snapshots[1].values.at("/ReplayInputs/Applied").value), 2.0);
        EXPECT_EQ(std::get<int64_t>(receiver.snapshots[1].values.at("/ReplayOutputs/Logger/QueuedCycles").value), 0);
        EXPECT_FALSE(Logger::GetReceiverQueueFault());

        Logger::End();
        EXPECT_EQ(receiver.endCalls, 1);
    }

    TEST_F(LoggerReplayParityTest, ReplayAfterUserPreservesReplayedSystemStats) {
        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) {
                    auto systemStats = table.GetSubtable("SystemStats");
                    systemStats.Put("BatteryVoltage", 12.34);
                    systemStats.Put("EpochTime", 123456789.0, "microseconds");
                    systemStats.GetSubtable("NTClients").GetSubtable("ReplayClient").Put("Connected", true);
                },
            },
        });

        Logger::SetReplaySource(&replaySource);
        Logger::Start();

        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/SystemStats/BatteryVoltage"));
        ASSERT_TRUE(values.contains("/SystemStats/EpochTime"));
        ASSERT_TRUE(values.contains("/SystemStats/NTClients/ReplayClient/Connected"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/SystemStats/BatteryVoltage").value), 12.34);
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/SystemStats/EpochTime").value), 123456789.0);
        EXPECT_TRUE(std::get<bool>(values.at("/SystemStats/NTClients/ReplayClient/Connected").value));

        Logger::End();
    }

    TEST_F(LoggerReplayParityTest, RealModeStillUsesLiveTimestampAndRealOutputs) {
        Logger::Start();

        EXPECT_TRUE(Logger::IsRunning());
        EXPECT_FALSE(Logger::HasReplaySource());
        EXPECT_GT(Logger::GetCurrentStorage().timestamp, 0);

        const int64_t monotonicTimeNs = wpi::RobotController::GetMonotonicTime();
        EXPECT_LE(std::llabs(Logger::GetCurrentStorage().timestamp - monotonicTimeNs), 100'000'000);

        Logger::RecordOutput("RealValue", 3.5);
        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/RealValue"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/RealOutputs/RealValue").value), 3.5);
    }

    TEST_F(LoggerReplayParityTest, DriverStationSaveUsesIntegerEncodingShape) {
        wpi::sim::DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::BLUE_2);
        wpi::sim::DriverStationSim::SetEventName("Week Zero");
        wpi::sim::DriverStationSim::SetGameData("  ABC  ");
        wpi::sim::DriverStationSim::SetMatchNumber(4);
        wpi::sim::DriverStationSim::SetReplayNumber(1);
        wpi::sim::DriverStationSim::SetMatchType(wpi::MatchType::QUALIFICATION);
        wpi::sim::DriverStationSim::SetMatchTime(12.5);
        wpi::sim::DriverStationSim::SetRobotMode(wpi::hal::RobotMode::TELEOPERATED);
        wpi::sim::DriverStationSim::SetJoystickName(0, "  Replay Pad  ");
        wpi::sim::DriverStationSim::SetJoystickGamepadType(0, 20);
        wpi::sim::DriverStationSim::SetJoystickIsGamepad(0, true);
        wpi::sim::DriverStationSim::SetJoystickButtonsAvailable(0, 0b111);
        wpi::sim::DriverStationSim::SetJoystickAxesAvailable(0, 0b11);
        wpi::sim::DriverStationSim::SetJoystickAxis(0, 0, 0.5);
        wpi::sim::DriverStationSim::SetJoystickAxis(0, 1, -1.0);
        wpi::sim::DriverStationSim::SetJoystickPOVsAvailable(0, 0b1);
        HALSIM_SetJoystickPOV(0, 0, HAL_JOYSTICK_POV_LEFT);
        const uint8_t fingerCounts[HAL_MAX_JOYSTICK_TOUCHPADS] = {2, 0};
        HALSIM_SetJoystickTouchpadCounts(0, 1, fingerCounts);
        HALSIM_SetJoystickTouchpadFinger(0, 0, 1, true, 0.75, 0.125);
        wpi::sim::DriverStationSim::NotifyNewData();
        DriverStationBackend::RefreshData();

        LogStorage storage;
        LogTable table(storage);
        akit::LoggedDriverStation::SaveToLog(table.GetSubtable("DriverStation"));

        const auto& values = storage.values;
        for (const char* removedKey :
             {"/DriverStation/GameSpecificMessage", "/DriverStation/Autonomous", "/DriverStation/Test", "/DriverStation/Joystick0/Xbox",
              "/DriverStation/Joystick0/ButtonCount", "/DriverStation/Joystick0/AxisTypes", "/DriverStation/Joystick0/POVs"}) {
            EXPECT_FALSE(values.contains(removedKey)) << removedKey;
        }

        EXPECT_EQ(values.at("/DriverStation/AllianceStation").type, akit::LoggableType::kInteger);
        EXPECT_EQ(values.at("/DriverStation/MatchType").type, akit::LoggableType::kInteger);
        EXPECT_EQ(values.at("/DriverStation/Joystick0/Type").type, akit::LoggableType::kInteger);
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/AllianceStation").value), HAL_ALLIANCE_STATION_BLUE_2);
        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/EventName").value), "Week Zero");
        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/GameData").value), "  ABC  ");
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/MatchType").value), 2);
        EXPECT_EQ(values.at("/DriverStation/MatchTime"), akit::LogValue(12.5, "", "seconds"));
        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/RobotMode").value), "TELEOPERATED");
        EXPECT_EQ(values.at("/DriverStation/OpModeId").type, akit::LoggableType::kInteger);
        EXPECT_EQ(values.at("/DriverStation/OpMode").type, akit::LoggableType::kString);

        EXPECT_EQ(std::get<std::string>(values.at("/DriverStation/Joystick0/Name").value), "Replay Pad");
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/Type").value), 20);
        EXPECT_TRUE(std::get<bool>(values.at("/DriverStation/Joystick0/IsGamepad").value));
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/ButtonsAvailable").value), 0b111);
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/AxesAvailable").value), 0b11);
        EXPECT_EQ(std::get<std::vector<float>>(values.at("/DriverStation/Joystick0/AxisValues").value), (std::vector<float>{0.5f, -1.0f}));
        EXPECT_EQ(values.at("/DriverStation/Joystick0/AxisRawValues").type, akit::LoggableType::kIntegerArray);
        EXPECT_EQ(std::get<std::vector<int64_t>>(values.at("/DriverStation/Joystick0/AxisRawValues").value).size(), 2u);
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/POVsAvailable").value), 0b1);
        EXPECT_EQ(std::get<std::vector<int64_t>>(values.at("/DriverStation/Joystick0/POVValues").value), (std::vector<int64_t>{HAL_JOYSTICK_POV_LEFT}));
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/TouchpadCount").value), 1);
        EXPECT_EQ(std::get<int64_t>(values.at("/DriverStation/Joystick0/Touchpad/0/FingerCount").value), 2);
        EXPECT_FALSE(std::get<bool>(values.at("/DriverStation/Joystick0/Touchpad/0/Finger/0/Down").value));
        EXPECT_TRUE(std::get<bool>(values.at("/DriverStation/Joystick0/Touchpad/0/Finger/1/Down").value));
        EXPECT_FLOAT_EQ(std::get<float>(values.at("/DriverStation/Joystick0/Touchpad/0/Finger/1/X").value), 0.75f);
        EXPECT_FLOAT_EQ(std::get<float>(values.at("/DriverStation/Joystick0/Touchpad/0/Finger/1/Y").value), 0.125f);
    }

} // namespace
