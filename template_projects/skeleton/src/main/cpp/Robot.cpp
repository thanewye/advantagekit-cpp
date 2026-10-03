// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "Robot.h"

#include <string>

#include <akit/LogFileUtil.h>
#include <akit/Logger.h>

#include "BuildConstants.h"
#include "Constants.h"

Robot::Robot() {
    // Record metadata
    akit::Logger::RecordMetadata("ProjectName", BuildConstants::MAVEN_NAME);
    akit::Logger::RecordMetadata("BuildDate", BuildConstants::BUILD_DATE);
    akit::Logger::RecordMetadata("GitSHA", BuildConstants::GIT_SHA);
    akit::Logger::RecordMetadata("GitDate", BuildConstants::GIT_DATE);
    akit::Logger::RecordMetadata("GitBranch", BuildConstants::GIT_BRANCH);
    switch (BuildConstants::DIRTY) {
    case 0:
        akit::Logger::RecordMetadata("GitDirty", "All changes committed");
        break;
    case 1:
        akit::Logger::RecordMetadata("GitDirty", "Uncommitted changes");
        break;
    default:
        akit::Logger::RecordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants::GetCurrentMode()) {
    case Constants::Mode::kReal:
        // Running on a real robot, log to a USB stick ("/U/logs")
        akit::Logger::AddDataReceiver(&wpilogWriter_.emplace());
        akit::Logger::AddDataReceiver(&nt4Publisher_.emplace());
        break;

    case Constants::Mode::kSim:
        // Running a physics simulator, log to NT
        akit::Logger::AddDataReceiver(&nt4Publisher_.emplace());
        break;

    case Constants::Mode::kReplay: {
        // Replaying a log, set up replay source
        SetUseTiming(false); // Run as fast as possible
        const std::string logPath = akit::LogFileUtil::FindReplayLog();
        akit::Logger::SetReplaySource(&replayReader_.emplace(logPath));
        akit::Logger::AddDataReceiver(&wpilogWriter_.emplace(akit::LogFileUtil::AddPathSuffix(logPath, "_sim")));
        break;
    }
    }

    // Start AdvantageKit logger
    akit::Logger::Start();
}

/** This function is called periodically during all modes. */
void Robot::RobotPeriodic() {}

/** This function is called once when the robot is disabled. */
void Robot::DisabledInit() {}

/** This function is called periodically when disabled. */
void Robot::DisabledPeriodic() {}

/** This function is called once when autonomous is enabled. */
void Robot::AutonomousInit() {}

/** This function is called periodically during autonomous. */
void Robot::AutonomousPeriodic() {}

/** This function is called once when teleop is enabled. */
void Robot::TeleopInit() {}

/** This function is called periodically during operator control. */
void Robot::TeleopPeriodic() {}

/** This function is called once when utility mode is enabled. */
void Robot::UtilityInit() {}

/** This function is called periodically during utility mode. */
void Robot::UtilityPeriodic() {}

/** This function is called once when the robot is first started up. */
void Robot::SimulationInit() {}

/** This function is called periodically whilst in simulation. */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_WPILIB_TESTS
int main() {
    return wpi::StartRobot<Robot>();
}
#endif
