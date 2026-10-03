// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "util/LocalADStarAK.h"

#include <akit/Logger.h>
#include <wpi/units/length.hpp>

bool LocalADStarAK::isNewPathAvailable() {
    if (!akit::Logger::HasReplaySource()) {
        io_.UpdateIsNewPathAvailable();
    }

    akit::Logger::ProcessInputs("LocalADStarAK", io_);

    return io_.isNewPathAvailable;
}

std::shared_ptr<pathplanner::PathPlannerPath> LocalADStarAK::getCurrentPath(pathplanner::PathConstraints constraints, pathplanner::GoalEndState goalEndState) {
    if (!akit::Logger::HasReplaySource()) {
        io_.UpdateCurrentPathPoints(constraints, goalEndState);
    }

    akit::Logger::ProcessInputs("LocalADStarAK", io_);

    if (io_.currentPathPoints.empty()) {
        return nullptr;
    }

    return pathplanner::PathPlannerPath::fromPathPoints(io_.currentPathPoints, constraints, goalEndState);
}

void LocalADStarAK::setStartPosition(const wpi::math::Translation2d& startPosition) {
    if (!akit::Logger::HasReplaySource()) {
        io_.adStar.setStartPosition(startPosition);
    }
}

void LocalADStarAK::setGoalPosition(const wpi::math::Translation2d& goalPosition) {
    if (!akit::Logger::HasReplaySource()) {
        io_.adStar.setGoalPosition(goalPosition);
    }
}

void LocalADStarAK::setDynamicObstacles(const std::vector<std::pair<wpi::math::Translation2d, wpi::math::Translation2d>>& obs,
                                        const wpi::math::Translation2d& currentRobotPos) {
    if (!akit::Logger::HasReplaySource()) {
        io_.adStar.setDynamicObstacles(obs, currentRobotPos);
    }
}

void LocalADStarAK::ADStarIO::ToLog(akit::LogTable& table) const {
    table.Put("IsNewPathAvailable", isNewPathAvailable);

    std::vector<double> pointsLogged;
    pointsLogged.reserve(currentPathPoints.size() * 2);
    for (const auto& point : currentPathPoints) {
        pointsLogged.push_back(point.position.X().value());
        pointsLogged.push_back(point.position.Y().value());
    }

    table.Put("CurrentPathPoints", std::span<const double>(pointsLogged));
}

void LocalADStarAK::ADStarIO::FromLog(const akit::LogTable& table) {
    isNewPathAvailable = table.Get("IsNewPathAvailable", false);

    std::vector<double> pointsLogged = table.Get("CurrentPathPoints", std::span<const double>{});

    std::vector<pathplanner::PathPoint> pathPoints;
    for (size_t i = 0; i + 1 < pointsLogged.size(); i += 2) {
        pathPoints.emplace_back(wpi::math::Translation2d{wpi::units::meter_t{pointsLogged[i]}, wpi::units::meter_t{pointsLogged[i + 1]}});
    }

    currentPathPoints = pathPoints;
}

void LocalADStarAK::ADStarIO::UpdateIsNewPathAvailable() {
    isNewPathAvailable = adStar.isNewPathAvailable();
}

void LocalADStarAK::ADStarIO::UpdateCurrentPathPoints(const pathplanner::PathConstraints& constraints, const pathplanner::GoalEndState& goalEndState) {
    auto currentPath = adStar.getCurrentPath(constraints, goalEndState);

    if (currentPath) {
        currentPathPoints = currentPath->getAllPathPoints();
    } else {
        currentPathPoints.clear();
    }
}
