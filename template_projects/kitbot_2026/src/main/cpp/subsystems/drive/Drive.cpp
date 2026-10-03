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
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <wpi/driverstation/MatchState.hpp>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/math/geometry/Twist2d.hpp>
#include <wpi/math/kinematics/DifferentialDriveWheelVelocities.hpp>

#include "Constants.h"
#include "util/LTVPathFollowingController.h"
#include "util/LocalADStarAK.h"

using namespace DriveConstants;

Drive::Drive(std::unique_ptr<DriveIO> io, std::unique_ptr<GyroIO> gyroIO)
    : io_(std::move(io))
    , gyroIO_(std::move(gyroIO))
    , kS_(Constants::GetCurrentMode() == Constants::Mode::kSim ? simKs : realKs)
    , kV_(Constants::GetCurrentMode() == Constants::Mode::kSim ? simKv : realKv)
    , sysId_(wpi::cmd::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                                     [](wpi::sysid::State state) { akit::Logger::RecordOutput("Drive/SysIdState", state); }},
             wpi::cmd::sysid::Mechanism{[this](wpi::units::volt_t voltage) { RunOpenLoop(voltage.value(), voltage.value()); }, nullptr, this}) {
    // Configure AutoBuilder for PathPlanner
    pathplanner::AutoBuilder::configure(
        [this] { return GetPose(); }, [this](const wpi::math::Pose2d& pose) { SetPose(pose); },
        [this] {
            return kinematics_.ToChassisVelocities(wpi::math::DifferentialDriveWheelVelocities{
                wpi::units::meters_per_second_t{GetLeftVelocityMetersPerSec()}, wpi::units::meters_per_second_t{GetRightVelocityMetersPerSec()}});
        },
        [this](const wpi::math::ChassisVelocities& speeds) { RunClosedLoop(speeds); }, std::make_shared<LTVPathFollowingController>(0.02_s), ppConfig,
        [] { return wpi::MatchState::GetAlliance().value_or(wpi::Alliance::BLUE) == wpi::Alliance::RED; }, this);
    pathplanner::Pathfinding::setPathfinder(std::make_unique<LocalADStarAK>());
    pathplanner::PathPlannerLogging::setLogActivePathCallback(
        [](const std::vector<wpi::math::Pose2d>& activePath) { akit::Logger::RecordOutput("Odometry/Trajectory", activePath); });
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback(
        [](const wpi::math::Pose2d& targetPose) { akit::Logger::RecordOutput("Odometry/TrajectorySetpoint", targetPose); });
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
        wpi::math::Twist2d twist = kinematics_.ToTwist2d(wpi::units::meter_t{GetLeftPositionMeters() - lastLeftPositionMeters_},
                                                         wpi::units::meter_t{GetRightPositionMeters() - lastRightPositionMeters_});
        rawGyroRotation_ = rawGyroRotation_ + wpi::math::Rotation2d{twist.dtheta};
        lastLeftPositionMeters_ = GetLeftPositionMeters();
        lastRightPositionMeters_ = GetRightPositionMeters();
    }

    // Update odometry
    poseEstimator_.Update(rawGyroRotation_, wpi::units::meter_t{GetLeftPositionMeters()}, wpi::units::meter_t{GetRightPositionMeters()});
}

void Drive::RunClosedLoop(const wpi::math::ChassisVelocities& speeds) {
    auto wheelSpeeds = kinematics_.ToWheelVelocities(speeds);
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

wpi::cmd::CommandPtr Drive::SysIdQuasistatic(wpi::cmd::sysid::Direction direction) {
    return sysId_.Quasistatic(direction);
}

wpi::cmd::CommandPtr Drive::SysIdDynamic(wpi::cmd::sysid::Direction direction) {
    return sysId_.Dynamic(direction);
}

wpi::math::Pose2d Drive::GetPose() const {
    return poseEstimator_.GetEstimatedPosition();
}

wpi::math::Rotation2d Drive::GetRotation() const {
    return GetPose().Rotation();
}

void Drive::SetPose(const wpi::math::Pose2d& pose) {
    poseEstimator_.ResetPosition(rawGyroRotation_, wpi::units::meter_t{GetLeftPositionMeters()}, wpi::units::meter_t{GetRightPositionMeters()}, pose);
}

void Drive::AddVisionMeasurement(const wpi::math::Pose2d& visionPose, wpi::units::second_t timestamp) {
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
