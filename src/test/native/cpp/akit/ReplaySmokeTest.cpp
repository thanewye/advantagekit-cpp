#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>
#include <units/time.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>

#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/wpilog/WPILOGReader.h"
#include "akit/wpilog/WPILOGWriter.h"

// Boost.PFR field-name reflection needs a type with linkage, so this can't live in the anonymous namespace
namespace replay_smoke_test {
    struct SmokeInputs {
        double position = 0.0;
    };
} // namespace replay_smoke_test

namespace {
    using akit::Logger;
    using akit::wpilog::WPILOGReader;
    using akit::wpilog::WPILOGWriter;
    using replay_smoke_test::SmokeInputs;

    constexpr size_t kCycleCount = 50;
    constexpr size_t kMaxReplayCycles = kCycleCount * 10;
    constexpr units::second_t kLoopPeriod{0.02};

    class ValidationLoggedRobot : public akit::LoggedRobot {};

    void EnsureLoggedRobotValidationSatisfied() {
        static ValidationLoggedRobot robot;
    }

    void ResetLoggerState() {
        Logger::End();
        Logger::ClearReceivers();
        Logger::SetReplaySource(nullptr);
        Logger::Clear();
        frc::sim::DriverStationSim::ResetData();
        frc::DriverStation::RefreshData();
    }

    std::vector<std::pair<int64_t, double>> ReadDoubleSeries(const std::string& path, std::string_view entryName) {
        std::vector<std::pair<int64_t, double>> series;
        auto buffer = wpi::MemoryBuffer::GetFile(path);
        if (!buffer) return series;

        wpi::log::DataLogReader reader(std::move(*buffer));
        std::optional<int> entryID;
        for (const auto& record : reader) {
            wpi::log::StartRecordData start;
            if (record.GetStartData(&start)) {
                if (start.name == entryName) entryID = start.entry;
            } else if (entryID && record.GetEntry() == *entryID) {
                double value;
                if (record.GetDouble(&value)) series.emplace_back(record.GetTimestamp(), value);
            }
        }
        return series;
    }

    void RunRealRobot(const std::string& outputPath) {
        WPILOGWriter writer(outputPath, WPILOGWriter::AdvantageScopeOpenBehavior::kNever);
        Logger::AddDataReceiver(&writer);
        Logger::Start();

        SmokeInputs inputs;
        for (size_t cycle = 0; cycle < kCycleCount; cycle++) {
            inputs.position = static_cast<double>(cycle) * 1.5;
            Logger::ProcessInputs("Smoke", inputs);
            Logger::RecordOutput("DoubledPosition", inputs.position * 2.0);
            Logger::PeriodicAfterUser();
            frc::sim::StepTimingAsync(kLoopPeriod);
            Logger::PeriodicBeforeUser();
        }

        Logger::End();
        Logger::ClearReceivers();
    }

    void RunReplay(const std::string& inputPath, const std::string& outputPath) {
        WPILOGReader reader(inputPath);
        WPILOGWriter writer(outputPath, WPILOGWriter::AdvantageScopeOpenBehavior::kNever);
        Logger::SetReplaySource(&reader);
        Logger::AddDataReceiver(&writer);
        Logger::Start();

        SmokeInputs inputs;
        for (size_t cycle = 0; Logger::IsRunning() && cycle < kMaxReplayCycles; cycle++) {
            Logger::ProcessInputs("Smoke", inputs);
            Logger::RecordOutput("DoubledPosition", inputs.position * 2.0);
            Logger::PeriodicAfterUser();
            Logger::PeriodicBeforeUser();
        }

        Logger::End();
        Logger::ClearReceivers();
        Logger::SetReplaySource(nullptr);
    }

    class ReplaySmokeTest : public ::testing::Test {
    protected:
        void SetUp() override {
            if (!frc::RobotBase::IsSimulation()) {
                GTEST_SKIP();
            }
            EnsureLoggedRobotValidationSatisfied();
            ResetLoggerState();
            frc::sim::PauseTiming();

            const auto tempDir = std::filesystem::temp_directory_path();
            realLogPath_ = (tempDir / "akit_smoke_real.wpilog").string();
            replayLogPath_ = (tempDir / "akit_smoke_replay.wpilog").string();
        }

        void TearDown() override {
            ResetLoggerState();
            frc::sim::ResumeTiming();
            std::error_code ec;
            std::filesystem::remove(realLogPath_, ec);
            std::filesystem::remove(replayLogPath_, ec);
        }

        std::string realLogPath_;
        std::string replayLogPath_;
    };

    TEST_F(ReplaySmokeTest, ReplayReproducesEveryCycleExceptFinalLikeJava) {
        RunRealRobot(realLogPath_);
        RunReplay(realLogPath_, replayLogPath_);

        const auto realInputs = ReadDoubleSeries(realLogPath_, "/Smoke/Position");
        const auto replayInputs = ReadDoubleSeries(replayLogPath_, "/Smoke/Position");
        const auto realOutputs = ReadDoubleSeries(realLogPath_, "/RealOutputs/DoubledPosition");
        const auto replayOutputs = ReadDoubleSeries(replayLogPath_, "/ReplayOutputs/DoubledPosition");

        ASSERT_EQ(realInputs.size(), kCycleCount);
        ASSERT_EQ(realOutputs.size(), kCycleCount);
        const std::vector realInputsWithoutFinalCycle(realInputs.begin(), realInputs.end() - 1);
        const std::vector realOutputsWithoutFinalCycle(realOutputs.begin(), realOutputs.end() - 1);
        EXPECT_EQ(replayInputs, realInputsWithoutFinalCycle);
        EXPECT_EQ(replayOutputs, realOutputsWithoutFinalCycle);
    }
} // namespace
