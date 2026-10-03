// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <akit/autolog/AutoLogOutput.h>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/math/estimator/DifferentialDrivePoseEstimator.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/DifferentialDriveKinematics.hpp>
#include <wpi/simulation/DifferentialDrivetrainSim.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/time.hpp>
#include <wpi/util/array.hpp>

/**
 * <b>IMPORTANT: This is a simple simulator for a differential drive, and has no support for real
 * hardware or IO implementations. It it intended only for simple testing in simulation.</b>
 *
 * <p>Please reference the other AdvantageKit template projects for more complete examples of drive
 * subsystems, including swerve and differential drive. Any subsystem with equivalent GetPose(),
 * GetRotation(), and AddVisionMeasurement() methods is compatible with this project's vision code.
 */
class DemoDrive : public wpi::cmd::SubsystemBase {
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
    wpi::math::Pose2d GetPose() const;

    /** Returns the latest estimated rotation from the pose estimator. */
    wpi::math::Rotation2d GetRotation() const;

    /** Adds a new timestamped vision measurement. */
    void AddVisionMeasurement(const wpi::math::Pose2d& visionRobotPoseMeters, wpi::units::second_t timestampSeconds,
                              const wpi::util::array<double, 3>& visionMeasurementStdDevs);

private:
    wpi::sim::DifferentialDrivetrainSim sim_ = wpi::sim::DifferentialDrivetrainSim::CreateKitbotSim(
        wpi::sim::DifferentialDrivetrainSim::KitbotMotor::DUAL_CIM_PER_SIDE, wpi::sim::DifferentialDrivetrainSim::KitbotGearing::RATIO_10P71,
        wpi::sim::DifferentialDrivetrainSim::KitbotWheelSize::SIX_INCH);
    wpi::math::DifferentialDriveKinematics kinematics_{26_in};
    wpi::math::DifferentialDrivePoseEstimator poseEstimator_{kinematics_, wpi::math::Rotation2d{}, 0_m, 0_m, wpi::math::Pose2d{}};

public:
    AUTOLOG_OUTPUT_SUPPLIER(pose, GetPose(), "EstimatedPose");
};
