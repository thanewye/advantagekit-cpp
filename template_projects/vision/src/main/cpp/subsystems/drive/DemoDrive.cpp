// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/DemoDrive.h"

void DemoDrive::Periodic() {
    sim_.Update(20_ms);
    poseEstimator_.Update(sim_.GetHeading(), sim_.GetLeftPosition(), sim_.GetRightPosition());
}

void DemoDrive::Run(double xAxis, double zAxis) {
    sim_.SetInputs(units::volt_t{(xAxis - zAxis) * 12.0}, units::volt_t{(xAxis + zAxis) * 12.0});
}

frc::Pose2d DemoDrive::GetPose() const {
    return poseEstimator_.GetEstimatedPosition();
}

frc::Rotation2d DemoDrive::GetRotation() const {
    return GetPose().Rotation();
}

void DemoDrive::AddVisionMeasurement(const frc::Pose2d& visionRobotPoseMeters, units::second_t timestampSeconds,
                                     const wpi::array<double, 3>& visionMeasurementStdDevs) {
    poseEstimator_.AddVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
}
