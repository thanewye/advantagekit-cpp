// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/vision/VisionIOPhotonVision.h"

#include <set>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <units/angle.h>

#include "subsystems/vision/VisionConstants.h"

using namespace VisionConstants;

VisionIOPhotonVision::VisionIOPhotonVision(std::string_view name, const frc::Transform3d& robotToCamera)
    : camera_(name)
    , robotToCamera_(robotToCamera) {}

void VisionIOPhotonVision::UpdateInputs(VisionIOInputs& inputs) {
    inputs.connected = camera_.IsConnected();

    // Read new camera observations
    std::set<int16_t> tagIds;
    std::vector<PoseObservation> poseObservations;
    for (auto& result : camera_.GetAllUnreadResults()) {
        // Update latest target observation
        if (result.HasTargets()) {
            inputs.latestTargetObservation = TargetObservation{frc::Rotation2d{units::degree_t{result.GetBestTarget().GetYaw()}},
                                                               frc::Rotation2d{units::degree_t{result.GetBestTarget().GetPitch()}}};
        } else {
            inputs.latestTargetObservation = TargetObservation{frc::Rotation2d{}, frc::Rotation2d{}};
        }

        // Add pose observation
        if (result.MultiTagResult().has_value()) { // Multitag result
            const auto& multitagResult = *result.MultiTagResult();

            // Calculate robot pose
            frc::Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            frc::Transform3d fieldToRobot = fieldToCamera + robotToCamera_.Inverse();
            frc::Pose3d robotPose{fieldToRobot.Translation(), fieldToRobot.Rotation()};

            // Calculate average tag distance
            double totalTagDistance = 0.0;
            for (const auto& target : result.GetTargets()) {
                totalTagDistance += target.GetBestCameraToTarget().Translation().Norm().value();
            }

            // Add tag IDs
            tagIds.insert(multitagResult.fiducialIDsUsed.begin(), multitagResult.fiducialIDsUsed.end());

            // Add observation
            poseObservations.push_back(PoseObservation{
                result.GetTimestamp().value(),                           // Timestamp
                robotPose,                                               // 3D pose estimate
                multitagResult.estimatedPose.ambiguity,                  // Ambiguity
                static_cast<int>(multitagResult.fiducialIDsUsed.size()), // Tag count
                totalTagDistance / result.GetTargets().size(),           // Average tag distance
                PoseObservationType::kPhotonVision});                    // Observation type

        } else if (!result.GetTargets().empty()) { // Single tag result
            const auto& target = result.GetTargets()[0];

            // Calculate robot pose
            auto tagPose = aprilTagLayout.GetTagPose(target.GetFiducialId());
            if (tagPose.has_value()) {
                frc::Transform3d fieldToTarget{tagPose->Translation(), tagPose->Rotation()};
                frc::Transform3d cameraToTarget = target.GetBestCameraToTarget();
                frc::Transform3d fieldToCamera = fieldToTarget + cameraToTarget.Inverse();
                frc::Transform3d fieldToRobot = fieldToCamera + robotToCamera_.Inverse();
                frc::Pose3d robotPose{fieldToRobot.Translation(), fieldToRobot.Rotation()};

                // Add tag ID
                tagIds.insert(static_cast<int16_t>(target.GetFiducialId()));

                // Add observation
                poseObservations.push_back(PoseObservation{
                    result.GetTimestamp().value(),               // Timestamp
                    robotPose,                                   // 3D pose estimate
                    target.GetPoseAmbiguity(),                   // Ambiguity
                    1,                                           // Tag count
                    cameraToTarget.Translation().Norm().value(), // Average tag distance
                    PoseObservationType::kPhotonVision});        // Observation type
            }
        }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations;

    // Save tag IDs to inputs objects
    inputs.tagIds.assign(tagIds.begin(), tagIds.end());
}
