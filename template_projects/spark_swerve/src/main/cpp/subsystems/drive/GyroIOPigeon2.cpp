// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/GyroIOPigeon2.h"

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <wpi/units/frequency.hpp>

#include "subsystems/drive/SparkOdometryThread.h"

using namespace DriveConstants;

GyroIOPigeon2::GyroIOPigeon2() {
    pigeon_.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});
    pigeon_.GetConfigurator().SetYaw(0_deg);
    yaw_.SetUpdateFrequency(wpi::units::hertz_t{odometryFrequency});
    yawVelocity_.SetUpdateFrequency(50_Hz);
    pigeon_.OptimizeBusUtilization();
    yawTimestampQueue_ = SparkOdometryThread::GetInstance().MakeTimestampQueue();
    auto yawClone = yaw_; // Status signals are not thread-safe
    yawPositionQueue_ = SparkOdometryThread::GetInstance().RegisterSignal([yawClone]() mutable { return yawClone.Refresh().GetValue().value(); });
}

void GyroIOPigeon2::UpdateInputs(GyroIOInputs& inputs) {
    inputs.connected = ctre::phoenix6::BaseStatusSignal::RefreshAll(yaw_, yawVelocity_).IsOK();
    inputs.yawPosition = wpi::math::Rotation2d{yaw_.GetValue()};
    inputs.yawVelocityRadPerSec = wpi::units::radians_per_second_t{yawVelocity_.GetValue()}.value();

    inputs.odometryYawTimestamps = yawTimestampQueue_->Drain();
    inputs.odometryYawPositions.clear();
    for (double value : yawPositionQueue_->Drain()) {
        inputs.odometryYawPositions.push_back(wpi::math::Rotation2d{wpi::units::degree_t{value}});
    }
}
