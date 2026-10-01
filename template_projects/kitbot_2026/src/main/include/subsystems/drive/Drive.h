// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <akit/autolog/AutoLogOutput.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/time.h>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"
#include "subsystems/drive/GyroIO.h"

class Drive : public frc2::SubsystemBase {
public:
    Drive(std::unique_ptr<DriveIO> io, std::unique_ptr<GyroIO> gyroIO);

    void Periodic() override;

    /** Runs the drive at the desired velocity. */
    void RunClosedLoop(const frc::ChassisSpeeds& speeds);

    /** Runs the drive at the desired left and right velocities. */
    void RunClosedLoop(double leftMetersPerSec, double rightMetersPerSec);

    /** Runs the drive in open loop. */
    void RunOpenLoop(double leftVolts, double rightVolts);

    /** Stops the drive. */
    void Stop();

    /** Returns a command to run a quasistatic test in the specified direction. */
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);

    /** Returns a command to run a dynamic test in the specified direction. */
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

    /** Returns the current odometry pose. */
    frc::Pose2d GetPose() const;

    /** Returns the current odometry rotation. */
    frc::Rotation2d GetRotation() const;

    /** Resets the current odometry pose. */
    void SetPose(const frc::Pose2d& pose);

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     */
    void AddVisionMeasurement(const frc::Pose2d& visionPose, units::second_t timestamp);

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

    frc::DifferentialDriveKinematics kinematics_{units::meter_t{DriveConstants::trackWidth}};
    double kS_;
    double kV_;
    frc::DifferentialDrivePoseEstimator poseEstimator_{kinematics_, frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{}};
    frc2::sysid::SysIdRoutine sysId_;
    frc::Rotation2d rawGyroRotation_{};
    double lastLeftPositionMeters_ = 0.0;
    double lastRightPositionMeters_ = 0.0;

public:
    AUTOLOG_OUTPUT_SUPPLIER(pose, GetPose(), "Odometry/Robot");
    AUTOLOG_OUTPUT_SUPPLIER(leftPositionMeters, GetLeftPositionMeters());
    AUTOLOG_OUTPUT_SUPPLIER(rightPositionMeters, GetRightPositionMeters());
    AUTOLOG_OUTPUT_SUPPLIER(leftVelocityMetersPerSec, GetLeftVelocityMetersPerSec());
    AUTOLOG_OUTPUT_SUPPLIER(rightVelocityMetersPerSec, GetRightVelocityMetersPerSec());
};
