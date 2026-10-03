// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <akit/autolog/AutoLogOutput.h>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/sysid/SysIdRoutine.hpp>
#include <wpi/math/estimator/DifferentialDrivePoseEstimator.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/DifferentialDriveKinematics.hpp>
#include <wpi/units/time.hpp>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"
#include "subsystems/drive/GyroIO.h"

class Drive : public wpi::cmd::SubsystemBase {
public:
    Drive(std::unique_ptr<DriveIO> io, std::unique_ptr<GyroIO> gyroIO);

    void Periodic() override;

    /** Runs the drive at the desired velocity. */
    void RunClosedLoop(const wpi::math::ChassisVelocities& speeds);

    /** Runs the drive at the desired left and right velocities. */
    void RunClosedLoop(double leftMetersPerSec, double rightMetersPerSec);

    /** Runs the drive in open loop. */
    void RunOpenLoop(double leftVolts, double rightVolts);

    /** Stops the drive. */
    void Stop();

    /** Returns a command to run a quasistatic test in the specified direction. */
    wpi::cmd::CommandPtr SysIdQuasistatic(wpi::cmd::sysid::Direction direction);

    /** Returns a command to run a dynamic test in the specified direction. */
    wpi::cmd::CommandPtr SysIdDynamic(wpi::cmd::sysid::Direction direction);

    /** Returns the current odometry pose. */
    wpi::math::Pose2d GetPose() const;

    /** Returns the current odometry rotation. */
    wpi::math::Rotation2d GetRotation() const;

    /** Resets the current odometry pose. */
    void SetPose(const wpi::math::Pose2d& pose);

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     */
    void AddVisionMeasurement(const wpi::math::Pose2d& visionPose, wpi::units::second_t timestamp);

    /** Returns the position of the left wheels in meters. */
    double GetLeftPositionMeters() const;

    /** Returns the position of the right wheels in meters. */
    double GetRightPositionMeters() const;

    /** Returns the velocity of the left wheels in meters/second. */
    double GetLeftVelocityMetersPerSec() const;

    /** Returns the velocity of the right wheels in meters/second. */
    double GetRightVelocityMetersPerSec() const;

    /** Returns the average velocity in radians/second. */
    double GetCharacterizationVelocity() const;

private:
    std::unique_ptr<DriveIO> io_;
    DriveIOInputs inputs_;
    std::unique_ptr<GyroIO> gyroIO_;
    GyroIOInputs gyroInputs_;

    wpi::math::DifferentialDriveKinematics kinematics_{wpi::units::meter_t{DriveConstants::trackWidth}};
    double kS_;
    double kV_;
    wpi::math::DifferentialDrivePoseEstimator poseEstimator_{wpi::math::Rotation2d{}, 0_m, 0_m, wpi::math::Pose2d{}};
    wpi::cmd::sysid::SysIdRoutine sysId_;
    wpi::math::Rotation2d rawGyroRotation_{};
    double lastLeftPositionMeters_ = 0.0;
    double lastRightPositionMeters_ = 0.0;

public:
    AUTOLOG_OUTPUT_SUPPLIER(pose, GetPose(), "Odometry/Robot");
    AUTOLOG_OUTPUT_SUPPLIER(leftPositionMeters, GetLeftPositionMeters());
    AUTOLOG_OUTPUT_SUPPLIER(rightPositionMeters, GetRightPositionMeters());
    AUTOLOG_OUTPUT_SUPPLIER(leftVelocityMetersPerSec, GetLeftVelocityMetersPerSec());
    AUTOLOG_OUTPUT_SUPPLIER(rightVelocityMetersPerSec, GetRightVelocityMetersPerSec());
};
