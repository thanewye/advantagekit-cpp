// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <akit/autolog/AutoLogOutput.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/array.h>

/**
 * <b>IMPORTANT: This is a simple simulator for a differential drive, and has no support for real
 * hardware or IO implementations. It it intended only for simple testing in simulation.</b>
 *
 * <p>Please reference the other AdvantageKit template projects for more complete examples of drive
 * subsystems, including swerve and differential drive. Any subsystem with equivalent GetPose(),
 * GetRotation(), and AddVisionMeasurement() methods is compatible with this project's vision code.
 */
class DemoDrive : public frc2::SubsystemBase {
public:
    void Periodic() override;

    /**
     * Drive open loop with percent out.
     *
     * @param xAxis The forward-back axis, where positive is forward.
     * @param zAxis The left-right axis, where positive is left.
     */
    void Run(double xAxis, double zAxis);

    /** Returns the latest estimated pose from the pose estimator. */
    frc::Pose2d GetPose() const;

    /** Returns the latest estimated rotation from the pose estimator. */
    frc::Rotation2d GetRotation() const;

    /** Adds a new timestamped vision measurement. */
    void AddVisionMeasurement(const frc::Pose2d& visionRobotPoseMeters, units::second_t timestampSeconds,
                              const wpi::array<double, 3>& visionMeasurementStdDevs);

private:
    frc::sim::DifferentialDrivetrainSim sim_ = frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
        frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,
        frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch);
    frc::DifferentialDriveKinematics kinematics_{26_in};
    frc::DifferentialDrivePoseEstimator poseEstimator_{kinematics_, frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{}};

public:
    AUTOLOG_OUTPUT_SUPPLIER(pose, GetPose(), "EstimatedPose");
};
