// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/vision/Vision.h"

#include <cmath>
#include <string>

#include <akit/Logger.h>
#include <wpi/math/geometry/Pose3d.hpp>

#include "subsystems/vision/VisionConstants.h"

using namespace VisionConstants;

wpi::math::Rotation2d Vision::GetTargetX(size_t cameraIndex) const {
    return inputs_[cameraIndex].latestTargetObservation.tx;
}

void Vision::Periodic() {
    for (size_t i = 0; i < io_.size(); i++) {
        io_[i]->UpdateInputs(inputs_[i]);
        akit::Logger::ProcessInputs("Vision/Camera" + std::to_string(i), inputs_[i]);
    }

    // Initialize logging values
    std::vector<wpi::math::Pose3d> allTagPoses;
    std::vector<wpi::math::Pose3d> allRobotPoses;
    std::vector<wpi::math::Pose3d> allRobotPosesAccepted;
    std::vector<wpi::math::Pose3d> allRobotPosesRejected;

    // Loop over cameras
    for (size_t cameraIndex = 0; cameraIndex < io_.size(); cameraIndex++) {
        // Update disconnected alert
        disconnectedAlerts_[cameraIndex]->Set(!inputs_[cameraIndex].connected);

        // Initialize logging values
        std::vector<wpi::math::Pose3d> tagPoses;
        std::vector<wpi::math::Pose3d> robotPoses;
        std::vector<wpi::math::Pose3d> robotPosesAccepted;
        std::vector<wpi::math::Pose3d> robotPosesRejected;

        // Add tag poses
        for (int tagId : inputs_[cameraIndex].tagIds) {
            auto tagPose = aprilTagLayout.GetTagPose(tagId);
            if (tagPose.has_value()) {
                tagPoses.push_back(*tagPose);
            }
        }

        // Loop over pose observations
        for (const auto& observation : inputs_[cameraIndex].poseObservations) {
            // Check whether to reject pose
            bool rejectPose = observation.tagCount == 0                                              // Must have at least one tag
                              || (observation.tagCount == 1 && observation.ambiguity > maxAmbiguity) // Cannot be high ambiguity
                              || std::abs(observation.pose.Z().value()) > maxZError                  // Must have realistic Z coordinate

                              // Must be within the field boundaries
                              || observation.pose.X() < 0.0_m || observation.pose.X() > aprilTagLayout.GetLength() || observation.pose.Y() < 0.0_m ||
                              observation.pose.Y() > aprilTagLayout.GetWidth();

            // Add pose to log
            robotPoses.push_back(observation.pose);
            if (rejectPose) {
                robotPosesRejected.push_back(observation.pose);
            } else {
                robotPosesAccepted.push_back(observation.pose);
            }

            // Skip if rejected
            if (rejectPose) {
                continue;
            }

            // Calculate standard deviations
            double stdDevFactor = std::pow(observation.averageTagDistance, 2.0) / observation.tagCount;
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double angularStdDev = angularStdDevBaseline * stdDevFactor;
            if (observation.type == PoseObservationType::kMegatag2) {
                linearStdDev *= linearStdDevMegatag2Factor;
                angularStdDev *= angularStdDevMegatag2Factor;
            }
            if (cameraIndex < cameraStdDevFactors.size()) {
                linearStdDev *= cameraStdDevFactors[cameraIndex];
                angularStdDev *= cameraStdDevFactors[cameraIndex];
            }

            // Send vision observation
            consumer_(observation.pose.ToPose2d(), wpi::units::second_t{observation.timestamp}, {linearStdDev, linearStdDev, angularStdDev});
        }

        // Log camera metadata
        akit::Logger::RecordOutput("Vision/Camera" + std::to_string(cameraIndex) + "/TagPoses", tagPoses);
        akit::Logger::RecordOutput("Vision/Camera" + std::to_string(cameraIndex) + "/RobotPoses", robotPoses);
        akit::Logger::RecordOutput("Vision/Camera" + std::to_string(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted);
        akit::Logger::RecordOutput("Vision/Camera" + std::to_string(cameraIndex) + "/RobotPosesRejected", robotPosesRejected);
        allTagPoses.insert(allTagPoses.end(), tagPoses.begin(), tagPoses.end());
        allRobotPoses.insert(allRobotPoses.end(), robotPoses.begin(), robotPoses.end());
        allRobotPosesAccepted.insert(allRobotPosesAccepted.end(), robotPosesAccepted.begin(), robotPosesAccepted.end());
        allRobotPosesRejected.insert(allRobotPosesRejected.end(), robotPosesRejected.begin(), robotPosesRejected.end());
    }

    // Log summary data
    akit::Logger::RecordOutput("Vision/Summary/TagPoses", allTagPoses);
    akit::Logger::RecordOutput("Vision/Summary/RobotPoses", allRobotPoses);
    akit::Logger::RecordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted);
    akit::Logger::RecordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected);
}
