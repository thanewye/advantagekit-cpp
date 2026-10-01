// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/GyroIOPigeon2.h"

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <units/frequency.h>

#include "subsystems/drive/Drive.h"
#include "subsystems/drive/PhoenixOdometryThread.h"

GyroIOPigeon2::GyroIOPigeon2() {
    if (TunerConstants::DrivetrainConstants.Pigeon2Configs.has_value()) {
        pigeon_.GetConfigurator().Apply(*TunerConstants::DrivetrainConstants.Pigeon2Configs);
    } else {
        pigeon_.GetConfigurator().Apply(configs::Pigeon2Configuration{});
    }

    pigeon_.GetConfigurator().SetYaw(0_deg);
    yaw_.SetUpdateFrequency(units::hertz_t{Drive::GetOdometryFrequency()});
    yawVelocity_.SetUpdateFrequency(50_Hz);
    pigeon_.OptimizeBusUtilization();
    yawTimestampQueue_ = PhoenixOdometryThread::GetInstance().MakeTimestampQueue();
    yawPositionQueue_ = PhoenixOdometryThread::GetInstance().RegisterSignal(yaw_);
}

void GyroIOPigeon2::UpdateInputs(GyroIOInputs& inputs) {
    inputs.connected = BaseStatusSignal::RefreshAll(yaw_, yawVelocity_).IsOK();
    inputs.yawPosition = frc::Rotation2d{yaw_.GetValue()};
    inputs.yawVelocityRadPerSec = units::radians_per_second_t{yawVelocity_.GetValue()}.value();

    inputs.odometryYawTimestamps = yawTimestampQueue_->Drain();
    inputs.odometryYawPositions.clear();
    for (double value : yawPositionQueue_->Drain()) {
        inputs.odometryYawPositions.push_back(frc::Rotation2d{units::degree_t{value}});
    }
}
