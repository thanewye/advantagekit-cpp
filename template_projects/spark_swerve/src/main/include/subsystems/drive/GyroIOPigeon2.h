// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/OdometryQueue.h"

/** IO implementation for Pigeon 2. */
class GyroIOPigeon2 : public GyroIO {
public:
    GyroIOPigeon2();

    void UpdateInputs(GyroIOInputs& inputs) override;

private:
    ctre::phoenix6::hardware::Pigeon2 pigeon_{DriveConstants::pigeonCanId, ctre::phoenix6::CANBus{wpi::CANPort::CAN_S0}};
    ctre::phoenix6::StatusSignal<wpi::units::degree_t> yaw_ = pigeon_.GetYaw();
    std::shared_ptr<OdometryQueue> yawPositionQueue_;
    std::shared_ptr<OdometryQueue> yawTimestampQueue_;
    ctre::phoenix6::StatusSignal<wpi::units::degrees_per_second_t> yawVelocity_ = pigeon_.GetAngularVelocityZWorld();
};
