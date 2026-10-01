// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <frc/filter/Debouncer.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

#include "generated/TunerConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/OdometryQueue.h"

/**
 * Module IO implementation for Talon FXS drive motor controller, Talon FXS turn motor controller,
 * and CANdi (PWM 1). Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
class ModuleIOTalonFXS : public ModuleIO {
public:
    using ModuleConstants =
        swerve::SwerveModuleConstants<configs::TalonFXSConfiguration, configs::TalonFXSConfiguration, configs::CANdiConfiguration>;

    explicit ModuleIOTalonFXS(const ModuleConstants& constants);

    void UpdateInputs(ModuleIOInputs& inputs) override;
    void SetDriveOpenLoop(double output) override;
    void SetTurnOpenLoop(double output) override;
    void SetDriveVelocity(double velocityRadPerSec) override;
    void SetTurnPosition(const frc::Rotation2d& rotation) override;

private:
    // Hardware objects
    hardware::TalonFXS driveTalon_;
    hardware::TalonFXS turnTalon_;
    hardware::CANdi candi_;

    // Voltage control requests
    controls::VoltageOut voltageRequest_{0_V};
    controls::PositionVoltage positionVoltageRequest_{0_tr};
    controls::VelocityVoltage velocityVoltageRequest_{0_tps};

    // Timestamp inputs from Phoenix thread
    std::shared_ptr<OdometryQueue> timestampQueue_;

    // Inputs from drive motor
    StatusSignal<units::turn_t> drivePosition_ = driveTalon_.GetPosition();
    std::shared_ptr<OdometryQueue> drivePositionQueue_;
    StatusSignal<units::turns_per_second_t> driveVelocity_ = driveTalon_.GetVelocity();
    StatusSignal<units::volt_t> driveAppliedVolts_ = driveTalon_.GetMotorVoltage();
    StatusSignal<units::ampere_t> driveCurrent_ = driveTalon_.GetStatorCurrent();

    // Inputs from turn motor
    StatusSignal<units::turn_t> turnAbsolutePosition_ = candi_.GetPWM1Position();
    StatusSignal<units::turn_t> turnPosition_ = turnTalon_.GetPosition();
    std::shared_ptr<OdometryQueue> turnPositionQueue_;
    StatusSignal<units::turns_per_second_t> turnVelocity_ = turnTalon_.GetVelocity();
    StatusSignal<units::volt_t> turnAppliedVolts_ = turnTalon_.GetMotorVoltage();
    StatusSignal<units::ampere_t> turnCurrent_ = turnTalon_.GetStatorCurrent();

    // Connection debouncers
    frc::Debouncer driveConnectedDebounce_{0.5_s, frc::Debouncer::DebounceType::kFalling};
    frc::Debouncer turnConnectedDebounce_{0.5_s, frc::Debouncer::DebounceType::kFalling};
    frc::Debouncer turnEncoderConnectedDebounce_{0.5_s, frc::Debouncer::DebounceType::kFalling};
};
