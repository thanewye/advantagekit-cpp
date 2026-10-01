// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <akit/inputs/LoggableInputs.h>
#include <frc/geometry/Translation2d.h>
#include <pathplanner/lib/path/GoalEndState.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/pathfinding/LocalADStar.h>
#include <pathplanner/lib/pathfinding/Pathfinder.h>

// NOTE: This file is ported from
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d

class LocalADStarAK : public pathplanner::Pathfinder {
public:
    /**
     * Get if a new path has been calculated since the last time a path was retrieved
     *
     * @return True if a new path is available
     */
    bool isNewPathAvailable() override;

    /**
     * Get the most recently calculated path
     *
     * @param constraints The path constraints to use when creating the path
     * @param goalEndState The goal end state to use when creating the path
     * @return The PathPlannerPath created from the points calculated by the pathfinder
     */
    std::shared_ptr<pathplanner::PathPlannerPath> getCurrentPath(pathplanner::PathConstraints constraints,
                                                                 pathplanner::GoalEndState goalEndState) override;

    /**
     * Set the start position to pathfind from
     *
     * @param startPosition Start position on the field. If this is within an obstacle it will be
     *     moved to the nearest non-obstacle node.
     */
    void setStartPosition(const frc::Translation2d& startPosition) override;

    /**
     * Set the goal position to pathfind to
     *
     * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
     *     to the nearest non-obstacle node.
     */
    void setGoalPosition(const frc::Translation2d& goalPosition) override;

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
     *     opposite corners of a bounding box.
     * @param currentRobotPos The current position of the robot. This is needed to change the start
     *     position of the path to properly avoid obstacles
     */
    void setDynamicObstacles(const std::vector<std::pair<frc::Translation2d, frc::Translation2d>>& obs,
                             const frc::Translation2d& currentRobotPos) override;

private:
    class ADStarIO : public akit::LoggableInputs {
    public:
        void ToLog(akit::LogTable& table) const override;
        void FromLog(const akit::LogTable& table) override;

        void UpdateIsNewPathAvailable();
        void UpdateCurrentPathPoints(const pathplanner::PathConstraints& constraints, const pathplanner::GoalEndState& goalEndState);

        pathplanner::LocalADStar adStar;
        bool isNewPathAvailable = false;
        std::vector<pathplanner::PathPoint> currentPathPoints;
    };

    ADStarIO io_;
};
