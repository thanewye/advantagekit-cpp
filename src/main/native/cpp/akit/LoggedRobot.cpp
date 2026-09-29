#include "pch.h"

#include "akit/LoggedRobot.h"

#include <exception>
#include <iostream>

#include <frc/RobotController.h>
#include <hal/DriverStation.h>
#include <hal/FRCUsageReporting.h>
#include <hal/Notifier.h>
#include <wpi/StackTrace.h>

#include "akit/Logger.h"

namespace akit {
    LoggedRobot::LoggedRobot(double period)
        : IterativeRobotBase(units::second_t{period})
        , periodUs_(static_cast<uint64_t>(period * 1'000'000)) {
        baseConstructed_ = true;
        int32_t status = 0;
        notifier_ = HAL_InitializeNotifier(&status);
        HAL_SetNotifierName(notifier_, "LoggedRobot", &status);
        HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_AdvantageKit);
        HAL_Report(HALUsageReporting::kResourceType_LoggingFramework, HALUsageReporting::kLoggingFramework_AdvantageKit);
    }

    LoggedRobot::~LoggedRobot() {
        int32_t status = 0;
        HAL_StopNotifier(notifier_, &status);
        HAL_CleanNotifier(notifier_);
    }

    void LoggedRobot::StartCompetition() {
        RobotInit();
        if (IsSimulation()) SimulationInit();
        uint64_t initEnd = frc::RobotController::GetFPGATime();

        Logger::PeriodicAfterUser(static_cast<int64_t>(initEnd), 0);

        std::cout << "********** Robot program startup complete **********" << std::endl;
        HAL_ObserveUserProgramStarting();

        try {
            while (true) {
                if (useTiming_) {
                    uint64_t now = frc::RobotController::GetFPGATime();
                    if (nextCycleUs_ < now) {
                        nextCycleUs_ = now;
                    } else {
                        int32_t status = 0;
                        HAL_UpdateNotifierAlarm(notifier_, nextCycleUs_, &status);
                        if (HAL_WaitForNotifierAlarm(notifier_, &status) == 0) {
                            Logger::End();
                            return;
                        }
                    }
                    nextCycleUs_ += periodUs_;
                }

                uint64_t beforeStart = frc::RobotController::GetFPGATime();
                Logger::PeriodicBeforeUser();
                if (!Logger::IsRunning()) {
                    return;
                }
                uint64_t userStart = frc::RobotController::GetFPGATime();
                LoopFunc();
                uint64_t userEnd = frc::RobotController::GetFPGATime();

                Logger::PeriodicAfterUser(static_cast<int64_t>(userEnd - userStart), static_cast<int64_t>(userStart - beforeStart));
            }
        } catch (const std::exception& e) {
            Logger::PeriodicAfterUser(0, 0, std::string{e.what()} + "\n" + wpi::GetStackTrace(0));
            Logger::End();
            throw;
        } catch (...) {
            Logger::PeriodicAfterUser(0, 0, wpi::GetStackTrace(0));
            Logger::End();
            throw;
        }
    }

    void LoggedRobot::EndCompetition() {
        int32_t status = 0;
        HAL_StopNotifier(notifier_, &status);
    }
} // namespace akit
