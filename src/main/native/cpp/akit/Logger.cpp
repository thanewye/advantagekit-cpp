#include "pch.h"

#include "akit/Logger.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <span>
#include <utility>

#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/util/MathShared.hpp>
#include <wpi/system/Errors.hpp>
#include <wpi/system/RobotController.hpp>
#include <wpi/system/Timer.hpp>

#include "akit/AlertLogger.h"
#include "akit/ConsoleSource.h"
#include "akit/LoggedRobot.h"
#include "akit/autolog/AutoLogOutputManager.h"
#include "akit/mechanism/LoggedMechanism2d.h"
#include "akit/networktables/LoggedNetworkInput.h"
#include "akit/telemetry/LoggedDriverStation.h"
#include "akit/telemetry/LoggedPowerDistribution.h"
#include "akit/telemetry/LoggedSystemStats.h"
#include "akit/telemetry/RadioLogger.h"

namespace {
    class TimerBasedMathShared : public wpi::math::MathShared {
    public:
        void ReportErrorV(std::string_view format, std::format_args args) override { wpi::ReportErrorV(wpi::err::Error, "", 0, "", format, args); }

        void ReportWarningV(std::string_view format, std::format_args args) override { wpi::ReportErrorV(wpi::warn::Warning, "", 0, "", format, args); }

        wpi::units::second_t GetTimestamp() override { return wpi::Timer::GetTimestamp(); }
    };
} // namespace

namespace akit {
    void Logger::Start() {
        if (running_) return;

        if (checkRobotBase_ && !LoggedRobot::IsBaseConstructed()) {
            WPILIB_ReportError(wpi::err::Error, "The main robot class must inherit from LoggedRobot when using AdvantageKit. For more details, check the "
                                                "AdvantageKit installation documentation: https://docs.advantagekit.org/getting-started/installation\n\n*** "
                                                "EXITING DUE TO INVALID ADVANTAGEKIT INSTALLATION, SEE ABOVE. ***");
            std::exit(1);
        }

        if (HasReplaySource()) {
            const char* halSimExtensions = std::getenv("HALSIM_EXTENSIONS");
            if (halSimExtensions != nullptr && halSimExtensions[0] != '\0') {
                WPILIB_ReportError(
                    wpi::err::Error,
                    "[AdvantageKit] All HAL simulation extensions must be disabled when running AdvantageKit replay, including the simulation GUI "
                    "and DriverStation connection. Check the configuration in \"build.gradle\" and ensure that all checkboxes are disabled in the "
                    "VSCode simulation popup.\n\n*** EXITING DUE TO INVALID SIMULATION CONFIGURATION, SEE ABOVE. ***");
                std::exit(1);
            }
        }

        if (console_ == nullptr) {
            if (wpi::RobotBase::IsReal()) console_ = std::make_unique<ConsoleSource::Systemcore>();
            else console_ = std::make_unique<ConsoleSource::Simulator>();
        }

        running_ = true;
        wpi::RobotController::SetTimeSource([]() -> int64_t { return static_cast<int64_t>(std::llround(GetTimestamp().value() * 1'000'000'000.0)); });
        wpi::math::MathSharedStore::SetMathShared(std::make_unique<TimerBasedMathShared>());
        lastTimestamp_ = wpi::RobotController::GetMonotonicTime();
        currentStorage_.Clear();
        currentStorage_.timestamp = 0;
        LogTable meta = LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayMetadata" : "RealMetadata");
        for (const auto& [k, v] : metadata_)
            meta.Put(k, v);
        if (HasReplaySource()) {
            replaySource_->Start();
            receiverThread_.StartSynchronous();
        } else {
            receiverThread_.Start();
        }
        PeriodicBeforeUser();
    }

    void Logger::End() {
        if (!running_) return;
        running_ = false;
        wpi::RobotController::SetTimeSource(wpi::RobotController::GetMonotonicTime);
        static_cast<void>(wpi::RobotController::GetTime());
        console_.reset();
        if (HasReplaySource()) {
            replaySource_->End();
        }
        receiverThread_.End();
        RadioLogger::Stop();
        LoggedPowerDistribution::Reset();
    }

    void Logger::PeriodicBeforeUser() {
        cycles_++;
        if (!running_) return;
        const int64_t entryUpdateStart = wpi::RobotController::GetMonotonicTime();
        LogTable root(currentStorage_);

        if (!HasReplaySource()) {
            currentStorage_.timestamp = wpi::RobotController::GetMonotonicTime();
        } else {
            if (!replaySource_->UpdateTable(root)) {
                End();
                return;
            }
        }

        const int64_t driverStationStart = wpi::RobotController::GetMonotonicTime();
        if (HasReplaySource()) {
            LoggedDriverStation::ReplayFromLog(root.GetSubtable("DriverStation"));
        }
        const int64_t dashboardInputsStart = wpi::RobotController::GetMonotonicTime();
        for (auto* input : dashboardInputs_) {
            if (input != nullptr) input->Periodic();
        }
        const int64_t dashboardInputsEnd = wpi::RobotController::GetMonotonicTime();

        RecordOutput("Logger/EntryUpdateMS", (driverStationStart - entryUpdateStart) / 1'000'000.0);
        if (HasReplaySource()) {
            RecordOutput("Logger/DriverStationMS", (dashboardInputsStart - driverStationStart) / 1'000'000.0);
        }
        RecordOutput("Logger/DashboardInputsMS", (dashboardInputsEnd - dashboardInputsStart) / 1'000'000.0);
    }

    void Logger::PeriodicAfterUser() {
        PeriodicAfterUser(0, 0);
    }

    void Logger::PeriodicAfterUser(const int64_t userCodeNs, const int64_t periodicBeforeNs) {
        PeriodicAfterUser(userCodeNs, periodicBeforeNs, "");
    }

    void Logger::PeriodicAfterUser(const int64_t userCodeNs, const int64_t periodicBeforeNs, const std::string_view extraConsoleData) {
        if (!running_) return;
        const int64_t afterStart = wpi::RobotController::GetMonotonicTime();

        LogTable root(currentStorage_);
        if (!HasReplaySource()) {
            const int64_t dsStart = wpi::RobotController::GetMonotonicTime();
            LogTable dsTable = root.GetSubtable("DriverStation");
            LoggedDriverStation::SaveToLog(dsTable);
            LoggedSystemStats::SaveToLog(root.GetSubtable("SystemStats"));
            if (const auto* loggedPowerDistribution = LoggedPowerDistribution::GetInstance()) {
                loggedPowerDistribution->SaveToLog(root.GetSubtable("PowerDistribution"));
            }
            RecordOutput("Logger/DriverStationMS", (wpi::RobotController::GetMonotonicTime() - dsStart) / 1'000'000.0);
        }
        const int64_t autoLogStart = wpi::RobotController::GetMonotonicTime();
        AutoLogOutputManager::Periodic();
        const int64_t alertStart = wpi::RobotController::GetMonotonicTime();
        AlertLogger::Periodic();
        const int64_t radioStart = wpi::RobotController::GetMonotonicTime();
        if (!HasReplaySource()) {
            RadioLogger::Periodic(root.GetSubtable("RadioStatus"), root.Get("SystemStats/TeamNumber", static_cast<int64_t>(-1)));
        }
        const int64_t consoleStart = wpi::RobotController::GetMonotonicTime();
        std::string consoleData = console_ != nullptr ? console_->GetNewData() : "";
        consoleData.append(extraConsoleData);
        if (!consoleData.empty()) {
            const size_t lastNonWhitespace = consoleData.find_last_not_of(" \t\r\n");
            if (lastNonWhitespace == std::string::npos) {
                consoleData.clear();
            } else {
                consoleData.erase(lastNonWhitespace + 1);
            }
            if (!consoleData.empty()) RecordOutput("Console", consoleData);
        }
        const int64_t consoleEnd = wpi::RobotController::GetMonotonicTime();

        RecordOutput("Logger/AutoLogMS", (alertStart - autoLogStart) / 1'000'000.0);
        RecordOutput("Logger/AlertLogMS", (radioStart - alertStart) / 1'000'000.0);
        RecordOutput("Logger/RadioLogMS", (consoleStart - radioStart) / 1'000'000.0);
        RecordOutput("Logger/ConsoleMS", (consoleEnd - consoleStart) / 1'000'000.0);

        LogTable loggerTable = root.GetSubtable("Logger");
        loggerTable.Put("Timestamp", currentStorage_.timestamp / 1'000'000'000.0);
        loggerTable.Put("TimeSinceLastCycle", (currentStorage_.timestamp - lastTimestamp_) / 1'000'000'000.0);
        loggerTable.Put("CycleCount", static_cast<int64_t>(cycles_));
        lastTimestamp_ = currentStorage_.timestamp;

        const int64_t periodicAfterNs = consoleEnd - afterStart;

        RecordOutput("LoggedRobot/UserCodeMS", userCodeNs / 1'000'000.0);
        RecordOutput("LoggedRobot/LogPeriodicMS", (periodicBeforeNs + periodicAfterNs) / 1'000'000.0);
        RecordOutput("LoggedRobot/FullCycleMS", (periodicBeforeNs + userCodeNs + periodicAfterNs) / 1'000'000.0);
        RecordOutput("Logger/QueuedCycles", static_cast<int64_t>(receiverThread_.QueueSize()));
        if (HasReplaySource()) {
            receiverThread_.DispatchNow(root);
            return;
        }
        LogStorage snapshot;
        LogTable::Clone(root, snapshot);
        if (!receiverThread_.Enqueue(std::move(snapshot))) {
            WPILIB_ReportError(wpi::err::Error, "[AdvantageKit] Capacity of receiver queue exceeded, data will NOT be logged.");
        }
    }

    void Logger::RecordOutput(const std::string& key, LogValue value) {
        if (!running_) return;

        LogTable outputs = LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs");
        outputs.Put(key, std::move(value));
    }

    void Logger::RecordOutput(const std::string& key, const bool value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const int64_t value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const float value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const double value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const std::string_view value) {
        RecordOutput(key, LogValue{std::string(value)});
    }

    void Logger::RecordOutput(const std::string& key, const mechanism::LoggedMechanism2d& value) {
        if (!running_) return;

        LogTable outputs = LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs");
        value.LogOutput(outputs.GetSubtable(key));
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const uint8_t> value) {
        RecordOutput(key, LogValue{std::vector<uint8_t>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const bool> value) {
        RecordOutput(key, LogValue{std::vector<bool>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::vector<bool>& value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const std::vector<bool>> value) {
        if (!running_) return;
        LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, value);
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const int> value) {
        RecordOutput(key, LogValue{std::vector<int64_t>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const int64_t> value) {
        RecordOutput(key, LogValue{std::vector<int64_t>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const float> value) {
        RecordOutput(key, LogValue{std::vector<float>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const double> value) {
        RecordOutput(key, LogValue{std::vector<double>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const std::string> value) {
        RecordOutput(key, LogValue{std::vector<std::string>(value.begin(), value.end())});
    }

    void Logger::RecordMetadata(const std::string& key, const std::string_view value) {
        if (running_) return;
        metadata_[std::string(key)] = std::string(value);
    }

    wpi::units::second_t Logger::GetTimestamp() {
        std::scoped_lock lock(mutex_);
        auto time = !running_ || currentStorage_.Empty() ? wpi::units::nanosecond_t{static_cast<double>(wpi::RobotController::GetMonotonicTime())}
                                                         : wpi::units::nanosecond_t{static_cast<double>(currentStorage_.timestamp)};
        return time;
    }

    void Logger::SetReplaySource(LogReplaySource* source) {
        if (running_) return;
        replaySource_ = source;
    }

    void Logger::RegisterDashboardInput(networktables::LoggedNetworkInput* dashboardInput) {
        if (dashboardInput == nullptr) return;
        if (std::find(dashboardInputs_.begin(), dashboardInputs_.end(), dashboardInput) == dashboardInputs_.end()) {
            dashboardInputs_.push_back(dashboardInput);
        }
    }

    void Logger::UnregisterDashboardInput(networktables::LoggedNetworkInput* dashboardInput) {
        dashboardInputs_.erase(std::remove(dashboardInputs_.begin(), dashboardInputs_.end(), dashboardInput), dashboardInputs_.end());
    }

    void Logger::SetConsoleSource(std::unique_ptr<ConsoleSource> console) {
        if (running_) return;
        console_ = std::move(console);
    }

    void Logger::DisableRobotBaseCheck() {
        checkRobotBase_ = false;
    }

    bool Logger::HasReplaySource() {
        return replaySource_ != nullptr;
    }

    void Logger::DumpCurrentStorage() {
        for (const auto& [key, lv] : currentStorage_.values) {
            std::cout << key << " = ";
            std::visit(
                []<typename T0>(const T0& v) {
                    using T = std::decay_t<T0>;
                    if constexpr (std::is_same_v<T, std::vector<double>> || std::is_same_v<T, std::vector<float>> || std::is_same_v<T, std::vector<int64_t>> ||
                                  std::is_same_v<T, std::vector<uint8_t>>) {
                        std::cout << "[";
                        for (size_t i = 0; i < v.size(); ++i)
                            std::cout << +v[i] << (i + 1 < v.size() ? ", " : "");
                        std::cout << "]";
                    } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
                        std::cout << "[";
                        for (size_t i = 0; i < v.size(); ++i)
                            std::cout << (v[i] ? "true" : "false") << (i + 1 < v.size() ? ", " : "");
                        std::cout << "]";
                    } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
                        std::cout << "[";
                        for (size_t i = 0; i < v.size(); ++i)
                            std::cout << "\"" << v[i] << "\"" << (i + 1 < v.size() ? ", " : "");
                        std::cout << "]";
                    } else {
                        std::cout << v;
                    }
                },
                lv.value);
            std::cout << "\n";
        }
    }

    void Logger::Clear() {
        AutoLogOutputManager::Reset();
        currentStorage_.Clear();
        currentStorage_.timestamp = 0;
    }

    const LogStorage& Logger::GetCurrentStorage() {
        return currentStorage_;
    }

    void Logger::AddDataReceiver(LogDataReceiver* receiver) {
        if (running_) {
            receiverThread_.AddDataReceiverAndCatchUp(receiver, LogTable(currentStorage_));
        } else {
            receiverThread_.AddDataReceiver(receiver);
        }
    }

    void Logger::ClearReceivers() {
        receiverThread_.ClearDataReceivers();
    }

    bool Logger::GetReceiverQueueFault() {
        return receiverThread_.HasFault();
    }
} // namespace akit
