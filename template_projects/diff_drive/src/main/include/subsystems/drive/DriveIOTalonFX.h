// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/DriveIO.h"

/** This drive implementation is for Talon FXs driving motors like the Falon 500 or Kraken X60. */
class DriveIOTalonFX : public DriveIO {
public:
    DriveIOTalonFX();

    void UpdateInputs(DriveIOInputs& inputs) override;
    void SetVoltage(double leftVolts, double rightVolts) override;
    void SetVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) override;

private:
    ctre::phoenix6::hardware::TalonFX leftLeader_{DriveConstants::leftLeaderCanId, ctre::phoenix6::CANBus{wpi::CANPort::CAN_S0}};
    ctre::phoenix6::hardware::TalonFX leftFollower_{DriveConstants::leftFollowerCanId, ctre::phoenix6::CANBus{wpi::CANPort::CAN_S0}};
    ctre::phoenix6::hardware::TalonFX rightLeader_{DriveConstants::rightLeaderCanId, ctre::phoenix6::CANBus{wpi::CANPort::CAN_S0}};
    ctre::phoenix6::hardware::TalonFX rightFollower_{DriveConstants::rightFollowerCanId, ctre::phoenix6::CANBus{wpi::CANPort::CAN_S0}};

    ctre::phoenix6::StatusSignal<wpi::units::turn_t> leftPosition_ = leftLeader_.GetPosition();
    ctre::phoenix6::StatusSignal<wpi::units::turns_per_second_t> leftVelocity_ = leftLeader_.GetVelocity();
    ctre::phoenix6::StatusSignal<wpi::units::volt_t> leftAppliedVolts_ = leftLeader_.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<wpi::units::ampere_t> leftLeaderCurrent_ = leftLeader_.GetSupplyCurrent();
    ctre::phoenix6::StatusSignal<wpi::units::ampere_t> leftFollowerCurrent_ = leftFollower_.GetSupplyCurrent();

    ctre::phoenix6::StatusSignal<wpi::units::turn_t> rightPosition_ = rightLeader_.GetPosition();
    ctre::phoenix6::StatusSignal<wpi::units::turns_per_second_t> rightVelocity_ = rightLeader_.GetVelocity();
    ctre::phoenix6::StatusSignal<wpi::units::volt_t> rightAppliedVolts_ = rightLeader_.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<wpi::units::ampere_t> rightLeaderCurrent_ = rightLeader_.GetSupplyCurrent();
    ctre::phoenix6::StatusSignal<wpi::units::ampere_t> rightFollowerCurrent_ = rightFollower_.GetSupplyCurrent();

    ctre::phoenix6::controls::VoltageOut voltageRequest_{0_V};
    ctre::phoenix6::controls::VelocityVoltage velocityRequest_{0_tps};
};
