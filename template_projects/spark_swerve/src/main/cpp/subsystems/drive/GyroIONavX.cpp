// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/GyroIONavX.h"

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "subsystems/drive/SparkOdometryThread.h"

GyroIONavX::GyroIONavX() {
    yawTimestampQueue_ = SparkOdometryThread::GetInstance().MakeTimestampQueue();
    yawPositionQueue_ = SparkOdometryThread::GetInstance().RegisterSignal([this] { return navX_.GetAngle(); });
}

void GyroIONavX::UpdateInputs(GyroIOInputs& inputs) {
    inputs.connected = navX_.IsConnected();
    inputs.yawPosition = frc::Rotation2d{units::degree_t{-navX_.GetAngle()}};
    inputs.yawVelocityRadPerSec = units::radians_per_second_t{units::degrees_per_second_t{-navX_.GetRawGyroZ()}}.value();

    inputs.odometryYawTimestamps = yawTimestampQueue_->Drain();
    inputs.odometryYawPositions.clear();
    for (double value : yawPositionQueue_->Drain()) {
        inputs.odometryYawPositions.push_back(frc::Rotation2d{units::degree_t{-value}});
    }
}
