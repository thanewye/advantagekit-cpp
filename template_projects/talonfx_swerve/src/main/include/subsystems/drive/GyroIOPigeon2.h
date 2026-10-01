// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "generated/TunerConstants.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/OdometryQueue.h"

/** IO implementation for Pigeon 2. */
class GyroIOPigeon2 : public GyroIO {
public:
    GyroIOPigeon2();

    void UpdateInputs(GyroIOInputs& inputs) override;

private:
    hardware::Pigeon2 pigeon_{TunerConstants::DrivetrainConstants.Pigeon2Id, TunerConstants::kCANBus};
    StatusSignal<units::degree_t> yaw_ = pigeon_.GetYaw();
    std::shared_ptr<OdometryQueue> yawPositionQueue_;
    std::shared_ptr<OdometryQueue> yawTimestampQueue_;
    StatusSignal<units::degrees_per_second_t> yawVelocity_ = pigeon_.GetAngularVelocityZWorld();
};
