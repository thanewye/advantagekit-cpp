// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <numbers>

#include <frc/controller/PIDController.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>

#include "generated/TunerConstants.h"
#include "subsystems/drive/ModuleIO.h"

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
class ModuleIOSim : public ModuleIO {
public:
    explicit ModuleIOSim(
        const swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>& constants);

    void UpdateInputs(ModuleIOInputs& inputs) override;
    void SetDriveOpenLoop(double output) override;
    void SetTurnOpenLoop(double output) override;
    void SetDriveVelocity(double velocityRadPerSec) override;
    void SetTurnPosition(const frc::Rotation2d& rotation) override;

private:
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    static constexpr double DRIVE_KP = 0.05;
    static constexpr double DRIVE_KD = 0.0;
    static constexpr double DRIVE_KS = 0.0;
    static constexpr double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    static constexpr double DRIVE_KV = 1.0 / (2 * std::numbers::pi / DRIVE_KV_ROT);
    static constexpr double TURN_KP = 8.0;
    static constexpr double TURN_KD = 0.0;
    static constexpr frc::DCMotor DRIVE_GEARBOX = frc::DCMotor::KrakenX60FOC(1);
    static constexpr frc::DCMotor TURN_GEARBOX = frc::DCMotor::KrakenX60FOC(1);

    frc::sim::DCMotorSim driveSim_;
    frc::sim::DCMotorSim turnSim_;

    bool driveClosedLoop_ = false;
    bool turnClosedLoop_ = false;
    frc::PIDController driveController_{DRIVE_KP, 0, DRIVE_KD};
    frc::PIDController turnController_{TURN_KP, 0, TURN_KD};
    double driveFFVolts_ = 0.0;
    double driveAppliedVolts_ = 0.0;
    double turnAppliedVolts_ = 0.0;
};
