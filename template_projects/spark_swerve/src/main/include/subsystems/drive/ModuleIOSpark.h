// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <wpi/math/filter/Debouncer.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>

#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/OdometryQueue.h"

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
class ModuleIOSpark : public ModuleIO {
public:
    explicit ModuleIOSpark(int module);

    void UpdateInputs(ModuleIOInputs& inputs) override;
    void SetDriveOpenLoop(double output) override;
    void SetTurnOpenLoop(double output) override;
    void SetDriveVelocity(double velocityRadPerSec) override;
    void SetTurnPosition(const wpi::math::Rotation2d& rotation) override;

private:
    wpi::math::Rotation2d zeroRotation_;

    // Hardware objects
    rev::spark::SparkFlex driveSpark_;
    rev::spark::SparkMax turnSpark_;
    rev::spark::SparkRelativeEncoder& driveEncoder_ = driveSpark_.GetEncoder();
    rev::spark::SparkAbsoluteEncoder& turnEncoder_ = turnSpark_.GetAbsoluteEncoder();

    // Closed loop controllers
    rev::spark::SparkClosedLoopController& driveController_ = driveSpark_.GetClosedLoopController();
    rev::spark::SparkClosedLoopController& turnController_ = turnSpark_.GetClosedLoopController();

    // Queue inputs from odometry thread
    std::shared_ptr<OdometryQueue> timestampQueue_;
    std::shared_ptr<OdometryQueue> drivePositionQueue_;
    std::shared_ptr<OdometryQueue> turnPositionQueue_;

    // Connection debouncers
    wpi::math::Debouncer driveConnectedDebounce_{0.5_s, wpi::math::Debouncer::DebounceType::FALLING};
    wpi::math::Debouncer turnConnectedDebounce_{0.5_s, wpi::math::Debouncer::DebounceType::FALLING};
};
