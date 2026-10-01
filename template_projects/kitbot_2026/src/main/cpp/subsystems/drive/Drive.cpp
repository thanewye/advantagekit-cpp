// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/Drive.h"

#include <cmath>
#include <vector>

#include <akit/Logger.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPLTVController.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include "Constants.h"
#include "util/LocalADStarAK.h"

using namespace DriveConstants;

Drive::Drive(std::unique_ptr<DriveIO> io, std::unique_ptr<GyroIO> gyroIO)
    : io_(std::move(io))
    , gyroIO_(std::move(gyroIO))
    , kS_(Constants::GetCurrentMode() == Constants::Mode::kSim ? simKs : realKs)
    , kV_(Constants::GetCurrentMode() == Constants::Mode::kSim ? simKv : realKv)
    , sysId_(frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                                 [](frc::sysid::State state) { akit::Logger::RecordOutput("Drive/SysIdState", state); }},
             frc2::sysid::Mechanism{[this](units::volt_t voltage) { RunOpenLoop(voltage.value(), voltage.value()); }, nullptr, this}) {
    // Configure AutoBuilder for PathPlanner
    pathplanner::AutoBuilder::configure(
        [this] { return GetPose(); }, [this](const frc::Pose2d& pose) { SetPose(pose); },
        [this] {
            return kinematics_.ToChassisSpeeds(frc::DifferentialDriveWheelSpeeds{units::meters_per_second_t{GetLeftVelocityMetersPerSec()},
                                                                                 units::meters_per_second_t{GetRightVelocityMetersPerSec()}});
        },
        [this](const frc::ChassisSpeeds& speeds) { RunClosedLoop(speeds); },
        std::make_shared<pathplanner::PPLTVController>(0.02_s, units::meters_per_second_t{maxSpeedMetersPerSec}), ppConfig,
        [] { return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed; }, this);
    pathplanner::Pathfinding::setPathfinder(std::make_unique<LocalADStarAK>());
    pathplanner::PathPlannerLogging::setLogActivePathCallback(
        [](const std::vector<frc::Pose2d>& activePath) { akit::Logger::RecordOutput("Odometry/Trajectory", activePath); });
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback(
        [](const frc::Pose2d& targetPose) { akit::Logger::RecordOutput("Odometry/TrajectorySetpoint", targetPose); });
}

void Drive::Periodic() {
    io_->UpdateInputs(inputs_);
    gyroIO_->UpdateInputs(gyroInputs_);
    akit::Logger::ProcessInputs("Drive", inputs_);
    akit::Logger::ProcessInputs("Drive/Gyro", gyroInputs_);

    // Update gyro angle
    if (gyroInputs_.connected) {
        // Use the real gyro angle
        rawGyroRotation_ = gyroInputs_.yawPosition;
    } else {
        // Use the angle delta from the kinematics and module deltas
        frc::Twist2d twist = kinematics_.ToTwist2d(units::meter_t{GetLeftPositionMeters() - lastLeftPositionMeters_},
                                                   units::meter_t{GetRightPositionMeters() - lastRightPositionMeters_});
        rawGyroRotation_ = rawGyroRotation_ + frc::Rotation2d{twist.dtheta};
        lastLeftPositionMeters_ = GetLeftPositionMeters();
        lastRightPositionMeters_ = GetRightPositionMeters();
    }

    // Update odometry
    poseEstimator_.Update(rawGyroRotation_, units::meter_t{GetLeftPositionMeters()}, units::meter_t{GetRightPositionMeters()});
}

void Drive::RunClosedLoop(const frc::ChassisSpeeds& speeds) {
    auto wheelSpeeds = kinematics_.ToWheelSpeeds(speeds);
    RunClosedLoop(wheelSpeeds.left.value(), wheelSpeeds.right.value());
}

void Drive::RunClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;
    akit::Logger::RecordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    akit::Logger::RecordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftFFVolts = kS_ * ((leftRadPerSec > 0) - (leftRadPerSec < 0)) + kV_ * leftRadPerSec;
    double rightFFVolts = kS_ * ((rightRadPerSec > 0) - (rightRadPerSec < 0)) + kV_ * rightRadPerSec;
    io_->SetVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
}

void Drive::RunOpenLoop(double leftVolts, double rightVolts) {
    io_->SetVoltage(leftVolts, rightVolts);
}

void Drive::Stop() {
    RunOpenLoop(0.0, 0.0);
}

frc2::CommandPtr Drive::SysIdQuasistatic(frc2::sysid::Direction direction) {
    return sysId_.Quasistatic(direction);
}

frc2::CommandPtr Drive::SysIdDynamic(frc2::sysid::Direction direction) {
    return sysId_.Dynamic(direction);
}

frc::Pose2d Drive::GetPose() const {
    return poseEstimator_.GetEstimatedPosition();
}

frc::Rotation2d Drive::GetRotation() const {
    return GetPose().Rotation();
}

void Drive::SetPose(const frc::Pose2d& pose) {
    poseEstimator_.ResetPosition(rawGyroRotation_, units::meter_t{GetLeftPositionMeters()}, units::meter_t{GetRightPositionMeters()}, pose);
}

void Drive::AddVisionMeasurement(const frc::Pose2d& visionPose, units::second_t timestamp) {
    poseEstimator_.AddVisionMeasurement(visionPose, timestamp);
}

double Drive::GetLeftPositionMeters() const {
    return inputs_.leftPositionRad * wheelRadiusMeters;
}

double Drive::GetRightPositionMeters() const {
    return inputs_.rightPositionRad * wheelRadiusMeters;
}

double Drive::GetLeftVelocityMetersPerSec() const {
    return inputs_.leftVelocityRadPerSec * wheelRadiusMeters;
}

double Drive::GetRightVelocityMetersPerSec() const {
    return inputs_.rightVelocityRadPerSec * wheelRadiusMeters;
}

double Drive::GetCharacterizationVelocity() const {
    return (inputs_.leftVelocityRadPerSec + inputs_.rightVelocityRadPerSec) / 2.0;
}
