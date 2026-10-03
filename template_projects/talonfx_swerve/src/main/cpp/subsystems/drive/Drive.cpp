// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/Drive.h"

#include <algorithm>
#include <cmath>
#include <span>

#include <akit/Logger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <wpi/commands2/Commands.hpp>
#include <wpi/driverstation/MatchState.hpp>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/math/geometry/Twist2d.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/util/UsageReporting.hpp>

#include "Constants.h"
#include "subsystems/drive/PhoenixOdometryThread.h"
#include "util/LocalADStarAK.h"

double Drive::GetOdometryFrequency() {
    static const double odometryFrequency = TunerConstants::kCANBus.IsNetworkFD() ? 250.0 : 100.0;
    return odometryFrequency;
}

const double Drive::DRIVE_BASE_RADIUS =
    std::max(std::max(std::hypot(TunerConstants::FrontLeft.LocationX.value(), TunerConstants::FrontLeft.LocationY.value()),
                      std::hypot(TunerConstants::FrontRight.LocationX.value(), TunerConstants::FrontRight.LocationY.value())),
             std::max(std::hypot(TunerConstants::BackLeft.LocationX.value(), TunerConstants::BackLeft.LocationY.value()),
                      std::hypot(TunerConstants::BackRight.LocationX.value(), TunerConstants::BackRight.LocationY.value())));

const pathplanner::RobotConfig Drive::PP_CONFIG{
    wpi::units::kilogram_t{ROBOT_MASS_KG}, wpi::units::kilogram_square_meter_t{ROBOT_MOI},
    pathplanner::ModuleConfig{TunerConstants::FrontLeft.WheelRadius, TunerConstants::kSpeedAt12Volts, WHEEL_COF,
                              wpi::math::DCMotor::KrakenX60FOC(1).WithReduction(TunerConstants::FrontLeft.DriveMotorGearRatio),
                              TunerConstants::FrontLeft.SlipCurrent, 1},
    [] {
        auto translations = GetModuleTranslations();
        return std::vector<wpi::math::Translation2d>(translations.begin(), translations.end());
    }()};

Drive::Drive(std::unique_ptr<GyroIO> gyroIO, std::unique_ptr<ModuleIO> flModuleIO, std::unique_ptr<ModuleIO> frModuleIO, std::unique_ptr<ModuleIO> blModuleIO,
             std::unique_ptr<ModuleIO> brModuleIO)
    : gyroIO_(std::move(gyroIO))
    , modules_{std::make_unique<Module>(std::move(flModuleIO), 0, TunerConstants::FrontLeft),
               std::make_unique<Module>(std::move(frModuleIO), 1, TunerConstants::FrontRight),
               std::make_unique<Module>(std::move(blModuleIO), 2, TunerConstants::BackLeft),
               std::make_unique<Module>(std::move(brModuleIO), 3, TunerConstants::BackRight)}
    , sysId_(wpi::cmd::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                                     [](wpi::sysid::State state) { akit::Logger::RecordOutput("Drive/SysIdState", state); }},
             wpi::cmd::sysid::Mechanism{[this](wpi::units::volt_t voltage) { RunCharacterization(voltage.value()); }, nullptr, this}) {
    // Usage reporting for swerve template
    wpi::util::ReportUsage("RobotDrive", "Swerve_AdvantageKit");

    // Start odometry thread
    PhoenixOdometryThread::GetInstance().Start();

    // Configure AutoBuilder for PathPlanner
    pathplanner::AutoBuilder::configure(
        [this] { return GetPose(); }, [this](const wpi::math::Pose2d& pose) { SetPose(pose); }, [this] { return GetChassisVelocities(); },
        [this](const wpi::math::ChassisVelocities& speeds) { RunVelocity(speeds); },
        std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants(5.0, 0.0, 0.0), pathplanner::PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG, [] { return wpi::MatchState::GetAlliance().value_or(wpi::Alliance::BLUE) == wpi::Alliance::RED; }, this);
    pathplanner::Pathfinding::setPathfinder(std::make_unique<LocalADStarAK>());
    pathplanner::PathPlannerLogging::setLogActivePathCallback(
        [](const std::vector<wpi::math::Pose2d>& activePath) { akit::Logger::RecordOutput("Odometry/Trajectory", activePath); });
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback(
        [](const wpi::math::Pose2d& targetPose) { akit::Logger::RecordOutput("Odometry/TrajectorySetpoint", targetPose); });
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
    if (wpi::RobotState::IsDisabled()) {
        for (auto& module : modules_) {
            module->Stop();
        }
    }

    // Log empty setpoint states when disabled
    if (wpi::RobotState::IsDisabled()) {
        akit::Logger::RecordOutput("SwerveStates/Setpoints", std::vector<wpi::math::SwerveModuleVelocity>{});
        akit::Logger::RecordOutput("SwerveStates/SetpointsOptimized", std::vector<wpi::math::SwerveModuleVelocity>{});
    }

    // Update odometry
    const std::vector<double>& sampleTimestamps = modules_[0]->GetOdometryTimestamps(); // All signals are sampled together
    size_t sampleCount = sampleTimestamps.size();
    for (size_t i = 0; i < sampleCount; i++) {
        // Read wheel positions and deltas from each module
        wpi::util::array<wpi::math::SwerveModulePosition, 4> modulePositions{wpi::util::empty_array};
        wpi::util::array<wpi::math::SwerveModulePosition, 4> moduleDeltas{wpi::util::empty_array};
        for (size_t moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            modulePositions[moduleIndex] = modules_[moduleIndex]->GetOdometryPositions()[i];
            moduleDeltas[moduleIndex] = wpi::math::SwerveModulePosition{modulePositions[moduleIndex].distance - lastModulePositions_[moduleIndex].distance,
                                                                        modulePositions[moduleIndex].angle};
            lastModulePositions_[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs_.connected) {
            // Use the real gyro angle
            rawGyroRotation_ = gyroInputs_.odometryYawPositions[i];
        } else {
            // Use the angle delta from the kinematics and module deltas
            wpi::math::Twist2d twist = kinematics_.ToTwist2d(moduleDeltas);
            rawGyroRotation_ = rawGyroRotation_ + wpi::math::Rotation2d{twist.dtheta};
        }

        // Apply update
        poseEstimator_.UpdateWithTime(wpi::units::second_t{sampleTimestamps[i]}, rawGyroRotation_, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert_.Set(!gyroInputs_.connected && Constants::GetCurrentMode() != Constants::Mode::kSim);
}

void Drive::RunVelocity(const wpi::math::ChassisVelocities& speeds) {
    // Calculate module setpoints
    wpi::math::ChassisVelocities discreteSpeeds = speeds.Discretize(0.02_s);
    auto setpointStates = kinematics_.ToSwerveModuleVelocities(discreteSpeeds);
    setpointStates = wpi::math::SwerveDriveKinematics<4>::DesaturateWheelVelocities(setpointStates, TunerConstants::kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    akit::Logger::RecordOutput("SwerveStates/Setpoints", std::span<const wpi::math::SwerveModuleVelocity>{setpointStates});
    akit::Logger::RecordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (size_t i = 0; i < 4; i++) {
        modules_[i]->RunSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (RunSetpoint mutates each state)
    akit::Logger::RecordOutput("SwerveStates/SetpointsOptimized", std::span<const wpi::math::SwerveModuleVelocity>{setpointStates});
}

void Drive::RunCharacterization(double output) {
    for (auto& module : modules_) {
        module->RunCharacterization(output);
    }
}

void Drive::Stop() {
    RunVelocity(wpi::math::ChassisVelocities{});
}

void Drive::StopWithX() {
    wpi::util::array<wpi::math::Rotation2d, 4> headings{wpi::util::empty_array};
    for (size_t i = 0; i < 4; i++) {
        headings[i] = GetModuleTranslations()[i].Angle().value_or(wpi::math::Rotation2d{});
    }
    kinematics_.ResetHeadings(headings);
    Stop();
}

wpi::cmd::CommandPtr Drive::SysIdQuasistatic(wpi::cmd::sysid::Direction direction) {
    return Run([this] { RunCharacterization(0.0); }).WithTimeout(1_s).AndThen(sysId_.Quasistatic(direction));
}

wpi::cmd::CommandPtr Drive::SysIdDynamic(wpi::cmd::sysid::Direction direction) {
    return Run([this] { RunCharacterization(0.0); }).WithTimeout(1_s).AndThen(sysId_.Dynamic(direction));
}

std::vector<wpi::math::SwerveModuleVelocity> Drive::GetModuleStates() const {
    std::vector<wpi::math::SwerveModuleVelocity> states;
    for (const auto& module : modules_) {
        states.push_back(module->GetState());
    }
    return states;
}

wpi::util::array<wpi::math::SwerveModulePosition, 4> Drive::GetModulePositions() const {
    wpi::util::array<wpi::math::SwerveModulePosition, 4> positions{wpi::util::empty_array};
    for (size_t i = 0; i < 4; i++) {
        positions[i] = modules_[i]->GetPosition();
    }
    return positions;
}

wpi::math::ChassisVelocities Drive::GetChassisVelocities() const {
    auto states = GetModuleStates();
    return kinematics_.ToChassisVelocities(wpi::util::array<wpi::math::SwerveModuleVelocity, 4>{states[0], states[1], states[2], states[3]});
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

wpi::math::Pose2d Drive::GetPose() const {
    return poseEstimator_.GetEstimatedPosition();
}

wpi::math::Rotation2d Drive::GetRotation() const {
    return GetPose().Rotation();
}

void Drive::SetPose(const wpi::math::Pose2d& pose) {
    poseEstimator_.ResetPosition(rawGyroRotation_, GetModulePositions(), pose);
}

void Drive::AddVisionMeasurement(const wpi::math::Pose2d& visionRobotPoseMeters, wpi::units::second_t timestampSeconds,
                                 const wpi::util::array<double, 3>& visionMeasurementStdDevs) {
    poseEstimator_.AddVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
}

double Drive::GetMaxLinearSpeedMetersPerSec() const {
    return TunerConstants::kSpeedAt12Volts.value();
}

double Drive::GetMaxAngularSpeedRadPerSec() const {
    return GetMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
}

wpi::util::array<wpi::math::Translation2d, 4> Drive::GetModuleTranslations() {
    return {wpi::math::Translation2d{TunerConstants::FrontLeft.LocationX, TunerConstants::FrontLeft.LocationY},
            wpi::math::Translation2d{TunerConstants::FrontRight.LocationX, TunerConstants::FrontRight.LocationY},
            wpi::math::Translation2d{TunerConstants::BackLeft.LocationX, TunerConstants::BackLeft.LocationY},
            wpi::math::Translation2d{TunerConstants::BackRight.LocationX, TunerConstants::BackRight.LocationY}};
}
