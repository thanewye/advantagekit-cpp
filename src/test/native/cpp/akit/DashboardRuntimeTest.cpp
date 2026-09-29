#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>

#include "akit/ConsoleSource.h"
#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/log/LogReplaySource.h"
#include "akit/log/LogTable.h"
#include "akit/networktables/LoggedDashboardChooser.h"
#include "akit/networktables/LoggedNetworkBoolean.h"
#include "akit/networktables/LoggedNetworkNumber.h"
#include "akit/networktables/LoggedNetworkString.h"

namespace {

    using akit::Logger;
    using akit::LogReplaySource;
    using akit::LogTable;
    using akit::networktables::LoggedDashboardChooser;
    using akit::networktables::LoggedNetworkBoolean;
    using akit::networktables::LoggedNetworkNumber;
    using akit::networktables::LoggedNetworkString;

    class ValidationLoggedRobot : public akit::LoggedRobot {
    public:
        ValidationLoggedRobot()
            : LoggedRobot() {}
    };

    void EnsureLoggedRobotValidationSatisfied() {
        static ValidationLoggedRobot robot;
    }

    class StubConsoleSource : public akit::ConsoleSource {
    public:
        StubConsoleSource() = default;

        explicit StubConsoleSource(std::vector<std::string> chunks)
            : chunks_(std::move(chunks)) {}

        [[nodiscard]] std::string GetNewData() override {
            if (nextChunk_ >= chunks_.size()) return "";
            return chunks_[nextChunk_++];
        }

    private:
        std::vector<std::string> chunks_;
        std::size_t nextChunk_ = 0;
    };

    struct ReplayFrame {
        int64_t timestamp;
        std::function<void(LogTable&)> apply;
    };

    class StubReplaySource : public LogReplaySource {
    public:
        explicit StubReplaySource(std::vector<ReplayFrame> frames)
            : frames_(std::move(frames)) {}

        bool UpdateTable(LogTable& table) override {
            if (nextFrame_ >= frames_.size()) return false;

            const auto& frame = frames_[nextFrame_++];
            table.SetTimestamp(frame.timestamp);
            if (frame.apply) frame.apply(table);
            return true;
        }

    private:
        std::vector<ReplayFrame> frames_;
        std::size_t nextFrame_ = 0;
    };

    void ResetLoggerState() {
        Logger::End();
        Logger::ClearReceivers();
        Logger::SetReplaySource(nullptr);
        Logger::SetConsoleSource(std::unique_ptr<akit::ConsoleSource>{});
        Logger::Clear();
        frc::sim::DriverStationSim::ResetData();
        frc::DriverStation::RefreshData();
    }

    class DashboardRuntimeTest : public ::testing::Test {
    protected:
        void SetUp() override {
            if (!frc::RobotBase::IsSimulation()) {
                GTEST_SKIP();
            }
            EnsureLoggedRobotValidationSatisfied();
            ResetLoggerState();
        }

        void TearDown() override { ResetLoggerState(); }

        void InstallNoopConsole() { Logger::SetConsoleSource(std::make_unique<StubConsoleSource>()); }
    };

    TEST_F(DashboardRuntimeTest, LoggedNetworkBooleanReplaysLoggedValueInsteadOfLiveNetworkTables) {
        LoggedNetworkBoolean input("/ReplayBoolean", false);
        nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/ReplayBoolean").GetEntry(false).Set(false);

        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").Put("ReplayBoolean", true); },
            },
            ReplayFrame{
                2'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").Put("ReplayBoolean", false); },
            },
        });

        Logger::SetReplaySource(&replaySource);
        InstallNoopConsole();
        Logger::Start();

        EXPECT_TRUE(input.Get());
        ASSERT_TRUE(Logger::GetCurrentStorage().values.contains("/NetworkInputs/ReplayBoolean"));
        EXPECT_TRUE(std::get<bool>(Logger::GetCurrentStorage().values.at("/NetworkInputs/ReplayBoolean").value));

        Logger::PeriodicBeforeUser();
        EXPECT_FALSE(input.Get());
        EXPECT_FALSE(std::get<bool>(Logger::GetCurrentStorage().values.at("/NetworkInputs/ReplayBoolean").value));

        Logger::End();
        Logger::SetReplaySource(nullptr);
    }

    TEST_F(DashboardRuntimeTest, LoggedNetworkNumberReplaysLoggedValueInsteadOfLiveNetworkTables) {
        LoggedNetworkNumber input("/ReplayNumber", 0.0);
        nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/ReplayNumber").GetEntry(0.0).Set(-1.0);

        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").Put("ReplayNumber", 42.5); },
            },
            ReplayFrame{
                2'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").Put("ReplayNumber", 7.25); },
            },
        });

        Logger::SetReplaySource(&replaySource);
        InstallNoopConsole();
        Logger::Start();

        EXPECT_DOUBLE_EQ(input.Get(), 42.5);
        ASSERT_TRUE(Logger::GetCurrentStorage().values.contains("/NetworkInputs/ReplayNumber"));
        EXPECT_DOUBLE_EQ(std::get<double>(Logger::GetCurrentStorage().values.at("/NetworkInputs/ReplayNumber").value), 42.5);

        Logger::PeriodicBeforeUser();
        EXPECT_DOUBLE_EQ(input.Get(), 7.25);
        EXPECT_DOUBLE_EQ(std::get<double>(Logger::GetCurrentStorage().values.at("/NetworkInputs/ReplayNumber").value), 7.25);

        Logger::End();
        Logger::SetReplaySource(nullptr);
    }

    TEST_F(DashboardRuntimeTest, LoggedNetworkStringReplaysLoggedValueInsteadOfLiveNetworkTables) {
        LoggedNetworkString input("/ReplayString", "default");
        nt::NetworkTableInstance::GetDefault().GetStringTopic("/ReplayString").GetEntry("default").Set("live");

        StubReplaySource replaySource({
            ReplayFrame{
                1'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").Put("ReplayString", "replay"); },
            },
            ReplayFrame{
                2'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").Put("ReplayString", "updated"); },
            },
        });

        Logger::SetReplaySource(&replaySource);
        InstallNoopConsole();
        Logger::Start();

        EXPECT_EQ(input.Get(), "replay");
        ASSERT_TRUE(Logger::GetCurrentStorage().values.contains("/NetworkInputs/ReplayString"));
        EXPECT_EQ(std::get<std::string>(Logger::GetCurrentStorage().values.at("/NetworkInputs/ReplayString").value), "replay");

        Logger::PeriodicBeforeUser();
        EXPECT_EQ(input.Get(), "updated");
        EXPECT_EQ(std::get<std::string>(Logger::GetCurrentStorage().values.at("/NetworkInputs/ReplayString").value), "updated");

        Logger::End();
        Logger::SetReplaySource(nullptr);
    }

    TEST_F(DashboardRuntimeTest, LoggedDashboardChooserCopiesOptionsFromExistingSendableChooser) {
        frc::SendableChooser<int> existing;
        existing.AddOption("Alternate", 2);
        existing.SetDefaultOption("Default", 1);

        LoggedDashboardChooser<int> chooser("WrappedChooser", existing);
        chooser.Periodic();

        EXPECT_EQ(chooser.Get(), 1);

        existing.AddOption("Extra", 3);
        EXPECT_EQ(chooser.GetSendableChooser().GetSelected(), "Default");
    }

    TEST_F(DashboardRuntimeTest, LoggedDashboardChooserUsesLiveSelectionAndPersistsReplaySelection) {
        LoggedDashboardChooser<int> chooser("ReplayChooser");
        chooser.AddDefaultOption("Default", 1);
        chooser.AddOption("Alternate", 2);
        frc::SmartDashboard::UpdateValues();

        auto selectedPublisher = nt::NetworkTableInstance::GetDefault().GetStringTopic("/SmartDashboard/ReplayChooser/selected").Publish();
        selectedPublisher.Set("Alternate");
        nt::NetworkTableInstance::GetDefault().FlushLocal();
        frc::SmartDashboard::UpdateValues();

        InstallNoopConsole();
        Logger::Start();

        EXPECT_EQ(chooser.Get(), 2);
        ASSERT_TRUE(Logger::GetCurrentStorage().values.contains("/NetworkInputs/SmartDashboard/ReplayChooser"));
        EXPECT_EQ(std::get<std::string>(Logger::GetCurrentStorage().values.at("/NetworkInputs/SmartDashboard/ReplayChooser").value), "Alternate");

        Logger::End();
        Logger::Clear();

        StubReplaySource replaySource({
            ReplayFrame{
                3'000,
                [](LogTable& table) { table.GetSubtable("NetworkInputs").GetSubtable("SmartDashboard").Put("ReplayChooser", "Alternate"); },
            },
            ReplayFrame{
                4'000,
                [](LogTable&) {},
            },
        });

        Logger::SetReplaySource(&replaySource);
        InstallNoopConsole();
        Logger::Start();

        EXPECT_EQ(chooser.Get(), 2);
        Logger::PeriodicBeforeUser();
        EXPECT_EQ(chooser.Get(), 2);
        EXPECT_EQ(std::get<std::string>(Logger::GetCurrentStorage().values.at("/NetworkInputs/SmartDashboard/ReplayChooser").value), "Alternate");

        Logger::End();
        Logger::SetReplaySource(nullptr);
    }

    TEST_F(DashboardRuntimeTest, AlertLoggerRecordsCanonicalOutputShape) {
        frc::Alert alert("DriveAlerts", "Motor hot", frc::Alert::AlertType::kWarning);
        alert.Set(true);
        frc::SmartDashboard::UpdateValues();
        nt::NetworkTableInstance::GetDefault().FlushLocal();

        InstallNoopConsole();
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/DriveAlerts/.type"));
        ASSERT_TRUE(values.contains("/RealOutputs/DriveAlerts/errors"));
        ASSERT_TRUE(values.contains("/RealOutputs/DriveAlerts/warnings"));
        ASSERT_TRUE(values.contains("/RealOutputs/DriveAlerts/infos"));
        EXPECT_EQ(std::get<std::string>(values.at("/RealOutputs/DriveAlerts/.type").value), "Alerts");
        EXPECT_TRUE(std::get<std::vector<std::string>>(values.at("/RealOutputs/DriveAlerts/errors").value).empty());
        EXPECT_EQ(std::get<std::vector<std::string>>(values.at("/RealOutputs/DriveAlerts/warnings").value), std::vector<std::string>{"Motor hot"});
        EXPECT_TRUE(std::get<std::vector<std::string>>(values.at("/RealOutputs/DriveAlerts/infos").value).empty());
    }

    TEST_F(DashboardRuntimeTest, LoggerMergesConsoleSourceDataWithExtraConsoleData) {
        Logger::SetConsoleSource(std::make_unique<StubConsoleSource>(std::vector<std::string>{"console line\nsecond line\n"}));
        Logger::Start();

        Logger::PeriodicAfterUser(0, 0, "extra line\n");

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/Console"));
        EXPECT_EQ(std::get<std::string>(values.at("/RealOutputs/Console").value), "console line\nsecond line\nextra line");
    }

    TEST(ConsoleSourceParityTest, SimulatorReturnsIncrementalStdoutAndStderrData) {
        if (!frc::RobotBase::IsSimulation()) GTEST_SKIP();

        akit::ConsoleSource::Simulator source;
        std::cout << "stdout-one" << std::flush;
        std::cerr << "stderr-one" << std::flush;
        EXPECT_EQ(source.GetNewData(), "stdout-onestderr-one");

        std::cout << "stdout-two" << std::flush;
        EXPECT_EQ(source.GetNewData(), "stdout-two");
        EXPECT_TRUE(source.GetNewData().empty());
    }

} // namespace
