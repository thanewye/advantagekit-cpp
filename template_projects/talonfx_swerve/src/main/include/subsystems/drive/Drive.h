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
#include <pathplanner/lib/config/RobotConfig.h>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/sysid/SysIdRoutine.hpp>
#include <wpi/math/estimator/SwerveDrivePoseEstimator.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/time.hpp>
#include <wpi/util/Alert.hpp>
#include <wpi/util/array.hpp>

#include "generated/TunerConstants.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/Module.h"
#include "subsystems/drive/ModuleIO.h"

class Drive : public wpi::cmd::SubsystemBase {
public:
    // TunerConstants doesn't include these constants, so they are declared locally
    static double GetOdometryFrequency();
    static const double DRIVE_BASE_RADIUS;

    inline static std::mutex odometryLock;

    Drive(std::unique_ptr<GyroIO> gyroIO, std::unique_ptr<ModuleIO> flModuleIO, std::unique_ptr<ModuleIO> frModuleIO, std::unique_ptr<ModuleIO> blModuleIO,
          std::unique_ptr<ModuleIO> brModuleIO);

    void Periodic() override;

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    void RunVelocity(const wpi::math::ChassisVelocities& speeds);

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
    wpi::cmd::CommandPtr SysIdQuasistatic(wpi::cmd::sysid::Direction direction);

    /** Returns a command to run a dynamic test in the specified direction. */
    wpi::cmd::CommandPtr SysIdDynamic(wpi::cmd::sysid::Direction direction);

    /** Returns the position of each module in radians. */
    std::array<double, 4> GetWheelRadiusCharacterizationPositions() const;

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    double GetFFCharacterizationVelocity() const;

    /** Returns the current odometry pose. */
    wpi::math::Pose2d GetPose() const;

    /** Returns the current odometry rotation. */
    wpi::math::Rotation2d GetRotation() const;

    /** Resets the current odometry pose. */
    void SetPose(const wpi::math::Pose2d& pose);

    /** Adds a new timestamped vision measurement. */
    void AddVisionMeasurement(const wpi::math::Pose2d& visionRobotPoseMeters, wpi::units::second_t timestampSeconds,
                              const wpi::util::array<double, 3>& visionMeasurementStdDevs);

    /** Returns the maximum linear speed in meters per sec. */
    double GetMaxLinearSpeedMetersPerSec() const;

    /** Returns the maximum angular speed in radians per sec. */
    double GetMaxAngularSpeedRadPerSec() const;

    /** Returns an array of module translations. */
    static wpi::util::array<wpi::math::Translation2d, 4> GetModuleTranslations();

private:
    // PathPlanner config constants
    static constexpr double ROBOT_MASS_KG = 74.088;
    static constexpr double ROBOT_MOI = 6.883;
    static constexpr double WHEEL_COF = 1.2;
    static const pathplanner::RobotConfig PP_CONFIG;

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    std::vector<wpi::math::SwerveModuleVelocity> GetModuleStates() const;

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    wpi::util::array<wpi::math::SwerveModulePosition, 4> GetModulePositions() const;

    /** Returns the measured chassis speeds of the robot. */
    wpi::math::ChassisVelocities GetChassisVelocities() const;

    std::unique_ptr<GyroIO> gyroIO_;
    GyroIOInputs gyroInputs_;
    std::array<std::unique_ptr<Module>, 4> modules_; // FL, FR, BL, BR
    wpi::cmd::sysid::SysIdRoutine sysId_;
    wpi::util::Alert gyroDisconnectedAlert_{"Drive/GyroDisconnected", "Disconnected gyro, using kinematics as fallback.", wpi::util::Alert::Level::HIGH};

    wpi::math::SwerveDriveKinematics<4> kinematics_{GetModuleTranslations()};
    wpi::math::Rotation2d rawGyroRotation_{};
    wpi::util::array<wpi::math::SwerveModulePosition, 4> lastModulePositions_{wpi::util::empty_array}; // For delta tracking
    wpi::math::SwerveDrivePoseEstimator<4> poseEstimator_{kinematics_, rawGyroRotation_, lastModulePositions_, wpi::math::Pose2d{}};

public:
    AUTOLOG_OUTPUT_SUPPLIER(moduleStates, GetModuleStates(), "SwerveStates/Measured");
    AUTOLOG_OUTPUT_SUPPLIER(chassisSpeeds, GetChassisVelocities(), "SwerveChassisSpeeds/Measured");
    AUTOLOG_OUTPUT_SUPPLIER(pose, GetPose(), "Odometry/Robot");
};
