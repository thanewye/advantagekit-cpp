#include "pch.h"

#include "akit/LoggedRobot.h"

#include <exception>
#include <iostream>

#include <wpi/driverstation/internal/DriverStationBackend.hpp>
#include <wpi/hal/Notifier.hpp>
#include <wpi/system/RobotController.hpp>
#include <wpi/util/StackTrace.hpp>
#include <wpi/util/Synchronization.hpp>
#include <wpi/util/UsageReporting.hpp>

#include "akit/Logger.h"

namespace akit {
    LoggedRobot::LoggedRobot(double period)
        : IterativeRobotBase(wpi::units::second_t{period})
        , periodNs_(static_cast<int64_t>(period * 1'000'000'000.0)) {
        baseConstructed_ = true;
        int32_t status = 0;
        notifier_ = HAL_CreateNotifier(&status);
        HAL_SetNotifierName(notifier_, "LoggedRobot", &status);
        wpi::util::ReportUsage("Framework", "AdvantageKit");
        wpi::util::ReportUsage("LoggingFramework", "AdvantageKit");
    }

    LoggedRobot::~LoggedRobot() {
        if (notifier_ != HAL_INVALID_HANDLE) HAL_DestroyNotifier(notifier_);
    }

    void LoggedRobot::StartCompetition() {
        const int64_t initStart = wpi::RobotController::GetMonotonicTime();
        if (IsSimulation()) SimulationInit();
        const int64_t initEnd = wpi::RobotController::GetMonotonicTime();

        Logger::PeriodicAfterUser(initEnd - initStart, 0);

        std::cout << "********** Robot program startup complete **********" << std::endl;
        wpi::internal::DriverStationBackend::ObserveUserProgramStarting();

        try {
            while (true) {
                if (useTiming_) {
                    const int64_t now = wpi::RobotController::GetMonotonicTime();
                    if (nextCycleNs_ < now) {
                        nextCycleNs_ = now;
                    } else {
                        int32_t status = 0;
                        HAL_SetNotifierAlarm(notifier_, nextCycleNs_, 0, true, true, &status);
                        if (!wpi::util::WaitForObject(notifier_)) {
                            Logger::End();
                            return;
                        }
                    }
                    nextCycleNs_ += periodNs_;
                }

                const int64_t beforeStart = wpi::RobotController::GetMonotonicTime();
                Logger::PeriodicBeforeUser();
                if (!Logger::IsRunning()) {
                    return;
                }
                const int64_t userStart = wpi::RobotController::GetMonotonicTime();
                LoopFunc();
                const int64_t userEnd = wpi::RobotController::GetMonotonicTime();

                Logger::PeriodicAfterUser(userEnd - userStart, userStart - beforeStart);
            }
        } catch (const std::exception& e) {
            Logger::PeriodicAfterUser(0, 0, std::string{e.what()} + "\n" + wpi::util::GetStackTrace(0));
            Logger::End();
            throw;
        } catch (...) {
            Logger::PeriodicAfterUser(0, 0, wpi::util::GetStackTrace(0));
            Logger::End();
            throw;
        }
    }

    void LoggedRobot::EndCompetition() {
        HAL_DestroyNotifier(notifier_);
        notifier_ = HAL_INVALID_HANDLE;
    }
} // namespace akit
