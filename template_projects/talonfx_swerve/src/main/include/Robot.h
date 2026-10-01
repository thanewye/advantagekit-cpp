// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <optional>

#include <akit/LoggedRobot.h>
#include <akit/networktables/NT4Publisher.h>
#include <akit/wpilog/WPILOGReader.h>
#include <akit/wpilog/WPILOGWriter.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class, you must also update the
 * main() function at the bottom of Robot.cpp.
 */
class Robot : public akit::LoggedRobot {
public:
    Robot();

    void RobotPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    std::optional<akit::wpilog::WPILOGWriter> wpilogWriter_;
    std::optional<akit::networktables::NT4Publisher> nt4Publisher_;
    std::optional<akit::wpilog::WPILOGReader> replayReader_;

    frc2::Command* autonomousCommand_ = nullptr;
    std::optional<RobotContainer> robotContainer_;
};
