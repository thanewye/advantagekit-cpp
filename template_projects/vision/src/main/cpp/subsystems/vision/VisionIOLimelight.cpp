// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/vision/VisionIOLimelight.h"

#include <set>

#include <frc/RobotController.h>
#include <frc/geometry/Rotation3d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>

VisionIOLimelight::VisionIOLimelight(std::string_view name, std::function<frc::Rotation2d()> rotationSupplier)
    : rotationSupplier_(std::move(rotationSupplier)) {
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(name);
    orientationPublisher_ = table->GetDoubleArrayTopic("robot_orientation_set").Publish();
    latencySubscriber_ = table->GetDoubleTopic("tl").Subscribe(0.0);
    txSubscriber_ = table->GetDoubleTopic("tx").Subscribe(0.0);
    tySubscriber_ = table->GetDoubleTopic("ty").Subscribe(0.0);
    megatag1Subscriber_ = table->GetDoubleArrayTopic("botpose_wpiblue").Subscribe({});
    megatag2Subscriber_ = table->GetDoubleArrayTopic("botpose_orb_wpiblue").Subscribe({});
}

void VisionIOLimelight::UpdateInputs(VisionIOInputs& inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected = ((static_cast<int64_t>(frc::RobotController::GetFPGATime()) - latencySubscriber_.GetLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation = TargetObservation{frc::Rotation2d{units::degree_t{txSubscriber_.Get()}},
                                                       frc::Rotation2d{units::degree_t{tySubscriber_.Get()}}};

    // Update orientation for MegaTag 2
    const std::vector<double> orientation{rotationSupplier_().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0};
    orientationPublisher_.Set(orientation);
    nt::NetworkTableInstance::GetDefault().Flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    std::set<int> tagIds;
    std::vector<PoseObservation> poseObservations;
    for (const auto& rawSample : megatag1Subscriber_.ReadQueue()) {
        if (rawSample.value.empty()) continue;
        for (size_t i = 11; i < rawSample.value.size(); i += 7) {
            tagIds.insert(static_cast<int>(rawSample.value[i]));
        }
        poseObservations.push_back(PoseObservation{
            // Timestamp, based on server timestamp of publish and latency
            rawSample.time * 1.0e-6 - rawSample.value[6] * 1.0e-3,

            // 3D pose estimate
            ParsePose(rawSample.value),

            // Ambiguity, using only the first tag because ambiguity isn't applicable for
            // multitag
            rawSample.value.size() >= 18 ? rawSample.value[17] : 0.0,

            // Tag count
            static_cast<int>(rawSample.value[7]),

            // Average tag distance
            rawSample.value[9],

            // Observation type
            PoseObservationType::kMegatag1});
    }
    for (const auto& rawSample : megatag2Subscriber_.ReadQueue()) {
        if (rawSample.value.empty()) continue;
        for (size_t i = 11; i < rawSample.value.size(); i += 7) {
            tagIds.insert(static_cast<int>(rawSample.value[i]));
        }
        poseObservations.push_back(PoseObservation{
            // Timestamp, based on server timestamp of publish and latency
            rawSample.time * 1.0e-6 - rawSample.value[6] * 1.0e-3,

            // 3D pose estimate
            ParsePose(rawSample.value),

            // Ambiguity, zeroed because the pose is already disambiguated
            0.0,

            // Tag count
            static_cast<int>(rawSample.value[7]),

            // Average tag distance
            rawSample.value[9],

            // Observation type
            PoseObservationType::kMegatag2});
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations;

    // Save tag IDs to inputs objects
    inputs.tagIds.assign(tagIds.begin(), tagIds.end());
}

frc::Pose3d VisionIOLimelight::ParsePose(const std::vector<double>& rawLLArray) {
    return frc::Pose3d{units::meter_t{rawLLArray[0]}, units::meter_t{rawLLArray[1]}, units::meter_t{rawLLArray[2]},
                       frc::Rotation3d{units::degree_t{rawLLArray[3]}, units::degree_t{rawLLArray[4]}, units::degree_t{rawLLArray[5]}}};
}
