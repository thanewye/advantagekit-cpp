// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <wpi/math/filter/Debouncer.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

#include "generated/TunerConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/OdometryQueue.h"

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
class ModuleIOTalonFX : public ModuleIO {
public:
    using ModuleConstants = swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>;

    explicit ModuleIOTalonFX(const ModuleConstants& constants);

    void UpdateInputs(ModuleIOInputs& inputs) override;
    void SetDriveOpenLoop(double output) override;
    void SetTurnOpenLoop(double output) override;
    void SetDriveVelocity(double velocityRadPerSec) override;
    void SetTurnPosition(const wpi::math::Rotation2d& rotation) override;

private:
    ModuleConstants constants_;

    // Hardware objects
    hardware::TalonFX driveTalon_;
    hardware::TalonFX turnTalon_;
    hardware::CANcoder cancoder_;

    // Voltage control requests
    controls::VoltageOut voltageRequest_{0_V};
    controls::PositionVoltage positionVoltageRequest_{0_tr};
    controls::VelocityVoltage velocityVoltageRequest_{0_tps};

    // Torque-current control requests
    controls::TorqueCurrentFOC torqueCurrentRequest_{0_A};
    controls::PositionTorqueCurrentFOC positionTorqueCurrentRequest_{0_tr};
    controls::VelocityTorqueCurrentFOC velocityTorqueCurrentRequest_{0_tps};

    // Timestamp inputs from Phoenix thread
    std::shared_ptr<OdometryQueue> timestampQueue_;

    // Inputs from drive motor
    StatusSignal<wpi::units::turn_t> drivePosition_ = driveTalon_.GetPosition();
    std::shared_ptr<OdometryQueue> drivePositionQueue_;
    StatusSignal<wpi::units::turns_per_second_t> driveVelocity_ = driveTalon_.GetVelocity();
    StatusSignal<wpi::units::volt_t> driveAppliedVolts_ = driveTalon_.GetMotorVoltage();
    StatusSignal<wpi::units::ampere_t> driveCurrent_ = driveTalon_.GetStatorCurrent();

    // Inputs from turn motor
    StatusSignal<wpi::units::turn_t> turnAbsolutePosition_ = cancoder_.GetAbsolutePosition();
    StatusSignal<wpi::units::turn_t> turnPosition_ = turnTalon_.GetPosition();
    std::shared_ptr<OdometryQueue> turnPositionQueue_;
    StatusSignal<wpi::units::turns_per_second_t> turnVelocity_ = turnTalon_.GetVelocity();
    StatusSignal<wpi::units::volt_t> turnAppliedVolts_ = turnTalon_.GetMotorVoltage();
    StatusSignal<wpi::units::ampere_t> turnCurrent_ = turnTalon_.GetStatorCurrent();

    // Connection debouncers
    wpi::math::Debouncer driveConnectedDebounce_{0.5_s, wpi::math::Debouncer::DebounceType::FALLING};
    wpi::math::Debouncer turnConnectedDebounce_{0.5_s, wpi::math::Debouncer::DebounceType::FALLING};
    wpi::math::Debouncer turnEncoderConnectedDebounce_{0.5_s, wpi::math::Debouncer::DebounceType::FALLING};
};
