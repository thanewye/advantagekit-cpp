// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include <akit/autolog/AutoLogOutput.h>
#include <frc/Alert.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/time.h>
#include <wpi/array.h>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/Module.h"
#include "subsystems/drive/ModuleIO.h"

class Drive : public frc2::SubsystemBase {
public:
    inline static std::mutex odometryLock;

    Drive(std::unique_ptr<GyroIO> gyroIO, std::unique_ptr<ModuleIO> flModuleIO, std::unique_ptr<ModuleIO> frModuleIO,
          std::unique_ptr<ModuleIO> blModuleIO, std::unique_ptr<ModuleIO> brModuleIO);

    void Periodic() override;

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    void RunVelocity(const frc::ChassisSpeeds& speeds);

    /** Runs the drive in a straight line with the specified drive output. */
    void RunCharacterization(double output);

    /** Stops the drive. */
    void Stop();

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    void StopWithX();

    /** Returns a command to run a quasistatic test in the specified direction. */
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);

    /** Returns a command to run a dynamic test in the specified direction. */
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

    /** Returns the position of each module in radians. */
    std::array<double, 4> GetWheelRadiusCharacterizationPositions() const;

    /** Returns the average velocity of the modules in rad/sec. */
    double GetFFCharacterizationVelocity() const;

    /** Returns the current odometry pose. */
    frc::Pose2d GetPose() const;

    /** Returns the current odometry rotation. */
    frc::Rotation2d GetRotation() const;

    /** Resets the current odometry pose. */
    void SetPose(const frc::Pose2d& pose);

    /** Adds a new timestamped vision measurement. */
    void AddVisionMeasurement(const frc::Pose2d& visionRobotPoseMeters, units::second_t timestampSeconds,
                              const wpi::array<double, 3>& visionMeasurementStdDevs);

    /** Returns the maximum linear speed in meters per sec. */
    double GetMaxLinearSpeedMetersPerSec() const;

    /** Returns the maximum angular speed in radians per sec. */
    double GetMaxAngularSpeedRadPerSec() const;

private:
    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    std::vector<frc::SwerveModuleState> GetModuleStates() const;

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    wpi::array<frc::SwerveModulePosition, 4> GetModulePositions() const;

    /** Returns the measured chassis speeds of the robot. */
    frc::ChassisSpeeds GetChassisSpeeds() const;

    std::unique_ptr<GyroIO> gyroIO_;
    GyroIOInputs gyroInputs_;
    std::array<std::unique_ptr<Module>, 4> modules_; // FL, FR, BL, BR
    frc2::sysid::SysIdRoutine sysId_;
    frc::Alert gyroDisconnectedAlert_{"Disconnected gyro, using kinematics as fallback.", frc::Alert::AlertType::kError};

    frc::SwerveDriveKinematics<4> kinematics_{DriveConstants::moduleTranslations};
    frc::Rotation2d rawGyroRotation_{};
    wpi::array<frc::SwerveModulePosition, 4> lastModulePositions_{wpi::empty_array}; // For delta tracking
    frc::SwerveDrivePoseEstimator<4> poseEstimator_{kinematics_, rawGyroRotation_, lastModulePositions_, frc::Pose2d{}};

public:
    AUTOLOG_OUTPUT_SUPPLIER(moduleStates, GetModuleStates(), "SwerveStates/Measured");
    AUTOLOG_OUTPUT_SUPPLIER(chassisSpeeds, GetChassisSpeeds(), "SwerveChassisSpeeds/Measured");
    AUTOLOG_OUTPUT_SUPPLIER(pose, GetPose(), "Odometry/Robot");
};
