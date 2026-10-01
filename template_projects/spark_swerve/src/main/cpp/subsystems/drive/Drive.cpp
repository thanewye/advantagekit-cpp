// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/Drive.h"

#include <span>

#include <akit/Logger.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Twist2d.h>
#include <frc2/command/Commands.h>
#include <hal/FRCUsageReporting.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include "Constants.h"
#include "subsystems/drive/SparkOdometryThread.h"
#include "util/LocalADStarAK.h"

using namespace DriveConstants;

Drive::Drive(std::unique_ptr<GyroIO> gyroIO, std::unique_ptr<ModuleIO> flModuleIO, std::unique_ptr<ModuleIO> frModuleIO,
             std::unique_ptr<ModuleIO> blModuleIO, std::unique_ptr<ModuleIO> brModuleIO)
    : gyroIO_(std::move(gyroIO))
    , modules_{std::make_unique<Module>(std::move(flModuleIO), 0), std::make_unique<Module>(std::move(frModuleIO), 1),
               std::make_unique<Module>(std::move(blModuleIO), 2), std::make_unique<Module>(std::move(brModuleIO), 3)}
    , sysId_(frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                                 [](frc::sysid::State state) { akit::Logger::RecordOutput("Drive/SysIdState", state); }},
             frc2::sysid::Mechanism{[this](units::volt_t voltage) { RunCharacterization(voltage.value()); }, nullptr, this}) {
    // Usage reporting for swerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread::GetInstance().Start();

    // Configure AutoBuilder for PathPlanner
    pathplanner::AutoBuilder::configure(
        [this] { return GetPose(); }, [this](const frc::Pose2d& pose) { SetPose(pose); }, [this] { return GetChassisSpeeds(); },
        [this](const frc::ChassisSpeeds& speeds) { RunVelocity(speeds); },
        std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants(5.0, 0.0, 0.0), pathplanner::PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        [] { return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed; }, this);
    pathplanner::Pathfinding::setPathfinder(std::make_unique<LocalADStarAK>());
    pathplanner::PathPlannerLogging::setLogActivePathCallback(
        [](const std::vector<frc::Pose2d>& activePath) { akit::Logger::RecordOutput("Odometry/Trajectory", activePath); });
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback(
        [](const frc::Pose2d& targetPose) { akit::Logger::RecordOutput("Odometry/TrajectorySetpoint", targetPose); });
}

void Drive::Periodic() {
    {
        std::lock_guard lock{odometryLock}; // Prevents odometry updates while reading data
        gyroIO_->UpdateInputs(gyroInputs_);
        akit::Logger::ProcessInputs("Drive/Gyro", gyroInputs_);
        for (auto& module : modules_) {
            module->Periodic();
        }
    }

    // Stop moving when disabled
    if (frc::DriverStation::IsDisabled()) {
        for (auto& module : modules_) {
            module->Stop();
        }
    }

    // Log empty setpoint states when disabled
    if (frc::DriverStation::IsDisabled()) {
        akit::Logger::RecordOutput("SwerveStates/Setpoints", std::vector<frc::SwerveModuleState>{});
        akit::Logger::RecordOutput("SwerveStates/SetpointsOptimized", std::vector<frc::SwerveModuleState>{});
    }

    // Update odometry
    const std::vector<double>& sampleTimestamps = modules_[0]->GetOdometryTimestamps(); // All signals are sampled together
    size_t sampleCount = sampleTimestamps.size();
    for (size_t i = 0; i < sampleCount; i++) {
        // Read wheel positions and deltas from each module
        wpi::array<frc::SwerveModulePosition, 4> modulePositions{wpi::empty_array};
        wpi::array<frc::SwerveModulePosition, 4> moduleDeltas{wpi::empty_array};
        for (size_t moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            modulePositions[moduleIndex] = modules_[moduleIndex]->GetOdometryPositions()[i];
            moduleDeltas[moduleIndex] = frc::SwerveModulePosition{
                modulePositions[moduleIndex].distance - lastModulePositions_[moduleIndex].distance, modulePositions[moduleIndex].angle};
            lastModulePositions_[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs_.connected) {
            // Use the real gyro angle
            rawGyroRotation_ = gyroInputs_.odometryYawPositions[i];
        } else {
            // Use the angle delta from the kinematics and module deltas
            frc::Twist2d twist = kinematics_.ToTwist2d(moduleDeltas);
            rawGyroRotation_ = rawGyroRotation_ + frc::Rotation2d{twist.dtheta};
        }

        // Apply update
        poseEstimator_.UpdateWithTime(units::second_t{sampleTimestamps[i]}, rawGyroRotation_, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert_.Set(!gyroInputs_.connected && Constants::GetCurrentMode() != Constants::Mode::kSim);
}

void Drive::RunVelocity(const frc::ChassisSpeeds& speeds) {
    // Calculate module setpoints
    frc::ChassisSpeeds discreteSpeeds = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);
    auto setpointStates = kinematics_.ToSwerveModuleStates(discreteSpeeds);
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&setpointStates, units::meters_per_second_t{maxSpeedMetersPerSec});

    // Log unoptimized setpoints
    akit::Logger::RecordOutput("SwerveStates/Setpoints", std::span<const frc::SwerveModuleState>{setpointStates});
    akit::Logger::RecordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (size_t i = 0; i < 4; i++) {
        modules_[i]->RunSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (RunSetpoint mutates each state)
    akit::Logger::RecordOutput("SwerveStates/SetpointsOptimized", std::span<const frc::SwerveModuleState>{setpointStates});
}

void Drive::RunCharacterization(double output) {
    for (auto& module : modules_) {
        module->RunCharacterization(output);
    }
}

void Drive::Stop() {
    RunVelocity(frc::ChassisSpeeds{});
}

void Drive::StopWithX() {
    wpi::array<frc::Rotation2d, 4> headings{wpi::empty_array};
    for (size_t i = 0; i < 4; i++) {
        headings[i] = moduleTranslations[i].Angle();
    }
    kinematics_.ResetHeadings(headings);
    Stop();
}

frc2::CommandPtr Drive::SysIdQuasistatic(frc2::sysid::Direction direction) {
    return Run([this] { RunCharacterization(0.0); }).WithTimeout(1_s).AndThen(sysId_.Quasistatic(direction));
}

frc2::CommandPtr Drive::SysIdDynamic(frc2::sysid::Direction direction) {
    return Run([this] { RunCharacterization(0.0); }).WithTimeout(1_s).AndThen(sysId_.Dynamic(direction));
}

std::vector<frc::SwerveModuleState> Drive::GetModuleStates() const {
    std::vector<frc::SwerveModuleState> states;
    for (const auto& module : modules_) {
        states.push_back(module->GetState());
    }
    return states;
}

wpi::array<frc::SwerveModulePosition, 4> Drive::GetModulePositions() const {
    wpi::array<frc::SwerveModulePosition, 4> positions{wpi::empty_array};
    for (size_t i = 0; i < 4; i++) {
        positions[i] = modules_[i]->GetPosition();
    }
    return positions;
}

frc::ChassisSpeeds Drive::GetChassisSpeeds() const {
    auto states = GetModuleStates();
    return kinematics_.ToChassisSpeeds(wpi::array<frc::SwerveModuleState, 4>{states[0], states[1], states[2], states[3]});
}

std::array<double, 4> Drive::GetWheelRadiusCharacterizationPositions() const {
    std::array<double, 4> values{};
    for (size_t i = 0; i < 4; i++) {
        values[i] = modules_[i]->GetWheelRadiusCharacterizationPosition();
    }
    return values;
}

double Drive::GetFFCharacterizationVelocity() const {
    double output = 0.0;
    for (const auto& module : modules_) {
        output += module->GetFFCharacterizationVelocity() / 4.0;
    }
    return output;
}

frc::Pose2d Drive::GetPose() const {
    return poseEstimator_.GetEstimatedPosition();
}

frc::Rotation2d Drive::GetRotation() const {
    return GetPose().Rotation();
}

void Drive::SetPose(const frc::Pose2d& pose) {
    poseEstimator_.ResetPosition(rawGyroRotation_, GetModulePositions(), pose);
}

void Drive::AddVisionMeasurement(const frc::Pose2d& visionRobotPoseMeters, units::second_t timestampSeconds,
                                 const wpi::array<double, 3>& visionMeasurementStdDevs) {
    poseEstimator_.AddVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
}

double Drive::GetMaxLinearSpeedMetersPerSec() const {
    return maxSpeedMetersPerSec;
}

double Drive::GetMaxAngularSpeedRadPerSec() const {
    return maxSpeedMetersPerSec / driveBaseRadius;
}
