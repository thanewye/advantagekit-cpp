// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/ModuleIOSpark.h"

#include <array>
#include <functional>

#include <frc/MathUtil.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/SparkOdometryThread.h"
#include "util/SparkUtil.h"

using namespace DriveConstants;
using namespace SparkUtil;

namespace {
    frc::Rotation2d ZeroRotationForModule(int module) {
        switch (module) {
            case 0: return frontLeftZeroRotation;
            case 1: return frontRightZeroRotation;
            case 2: return backLeftZeroRotation;
            case 3: return backRightZeroRotation;
            default: return frc::Rotation2d{};
        }
    }

    int DriveCanIdForModule(int module) {
        switch (module) {
            case 0: return frontLeftDriveCanId;
            case 1: return frontRightDriveCanId;
            case 2: return backLeftDriveCanId;
            case 3: return backRightDriveCanId;
            default: return 0;
        }
    }

    int TurnCanIdForModule(int module) {
        switch (module) {
            case 0: return frontLeftTurnCanId;
            case 1: return frontRightTurnCanId;
            case 2: return backLeftTurnCanId;
            case 3: return backRightTurnCanId;
            default: return 0;
        }
    }
} // namespace

ModuleIOSpark::ModuleIOSpark(int module)
    : zeroRotation_(ZeroRotationForModule(module))
    , driveSpark_(DriveCanIdForModule(module), rev::spark::SparkFlex::MotorType::kBrushless)
    , turnSpark_(TurnCanIdForModule(module), rev::spark::SparkMax::MotorType::kBrushless) {
    // Configure drive motor
    rev::spark::SparkFlexConfig driveConfig;
    driveConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake).SmartCurrentLimit(driveMotorCurrentLimit).VoltageCompensation(12.0);
    driveConfig.encoder.PositionConversionFactor(driveEncoderPositionFactor)
        .VelocityConversionFactor(driveEncoderVelocityFactor)
        .UvwMeasurementPeriod(10)
        .UvwAverageDepth(2);
    driveConfig.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder).Pid(driveKp, 0.0, driveKd);
    driveConfig.signals.PrimaryEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(static_cast<int>(1000.0 / odometryFrequency))
        .PrimaryEncoderVelocityAlwaysOn(true)
        .PrimaryEncoderVelocityPeriodMs(20)
        .AppliedOutputPeriodMs(20)
        .BusVoltagePeriodMs(20)
        .OutputCurrentPeriodMs(20);
    TryUntilOk(driveSpark_, 5, [&] {
        return driveSpark_.Configure(driveConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    });
    TryUntilOk(driveSpark_, 5, [&] { return driveEncoder_.SetPosition(0.0); });

    // Configure turn motor
    rev::spark::SparkMaxConfig turnConfig;
    turnConfig.Inverted(turnInverted)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(turnMotorCurrentLimit)
        .VoltageCompensation(12.0);
    turnConfig.absoluteEncoder.Inverted(turnEncoderInverted)
        .PositionConversionFactor(turnEncoderPositionFactor)
        .VelocityConversionFactor(turnEncoderVelocityFactor)
        .AverageDepth(2);
    turnConfig.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .Pid(turnKp, 0.0, turnKd);
    turnConfig.signals.AbsoluteEncoderPositionAlwaysOn(true)
        .AbsoluteEncoderPositionPeriodMs(static_cast<int>(1000.0 / odometryFrequency))
        .AbsoluteEncoderVelocityAlwaysOn(true)
        .AbsoluteEncoderVelocityPeriodMs(20)
        .AppliedOutputPeriodMs(20)
        .BusVoltagePeriodMs(20)
        .OutputCurrentPeriodMs(20);
    TryUntilOk(turnSpark_, 5, [&] {
        return turnSpark_.Configure(turnConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    });

    // Create odometry queues
    timestampQueue_ = SparkOdometryThread::GetInstance().MakeTimestampQueue();
    drivePositionQueue_ = SparkOdometryThread::GetInstance().RegisterSignal(driveSpark_, [this] { return driveEncoder_.GetPosition(); });
    turnPositionQueue_ = SparkOdometryThread::GetInstance().RegisterSignal(turnSpark_, [this] { return turnEncoder_.GetPosition(); });
}

void ModuleIOSpark::UpdateInputs(ModuleIOInputs& inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    IfOk(driveSpark_, [&] { return driveEncoder_.GetPosition(); }, [&](double value) { inputs.drivePositionRad = value; });
    IfOk(driveSpark_, [&] { return driveEncoder_.GetVelocity(); }, [&](double value) { inputs.driveVelocityRadPerSec = value; });
    const std::array<std::function<double()>, 2> driveVoltageSuppliers{[&] { return driveSpark_.GetAppliedOutput(); },
                                                                       [&] { return driveSpark_.GetBusVoltage(); }};
    IfOk(driveSpark_, driveVoltageSuppliers, [&](const std::vector<double>& values) { inputs.driveAppliedVolts = values[0] * values[1]; });
    IfOk(driveSpark_, [&] { return driveSpark_.GetOutputCurrent(); }, [&](double value) { inputs.driveCurrentAmps = value; });
    inputs.driveConnected = driveConnectedDebounce_.Calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    IfOk(turnSpark_, [&] { return turnEncoder_.GetPosition(); },
         [&](double value) { inputs.turnPosition = frc::Rotation2d{units::radian_t{value}} - zeroRotation_; });
    IfOk(turnSpark_, [&] { return turnEncoder_.GetVelocity(); }, [&](double value) { inputs.turnVelocityRadPerSec = value; });
    const std::array<std::function<double()>, 2> turnVoltageSuppliers{[&] { return turnSpark_.GetAppliedOutput(); },
                                                                      [&] { return turnSpark_.GetBusVoltage(); }};
    IfOk(turnSpark_, turnVoltageSuppliers, [&](const std::vector<double>& values) { inputs.turnAppliedVolts = values[0] * values[1]; });
    IfOk(turnSpark_, [&] { return turnSpark_.GetOutputCurrent(); }, [&](double value) { inputs.turnCurrentAmps = value; });
    inputs.turnConnected = turnConnectedDebounce_.Calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue_->Drain();
    inputs.odometryDrivePositionsRad = drivePositionQueue_->Drain();
    inputs.odometryTurnPositions.clear();
    for (double value : turnPositionQueue_->Drain()) {
        inputs.odometryTurnPositions.push_back(frc::Rotation2d{units::radian_t{value}} - zeroRotation_);
    }
}

void ModuleIOSpark::SetDriveOpenLoop(double output) {
    driveSpark_.SetVoltage(units::volt_t{output});
}

void ModuleIOSpark::SetTurnOpenLoop(double output) {
    turnSpark_.SetVoltage(units::volt_t{output});
}

void ModuleIOSpark::SetDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * ((velocityRadPerSec > 0) - (velocityRadPerSec < 0)) + driveKv * velocityRadPerSec;
    driveController_.SetSetpoint(velocityRadPerSec, rev::spark::SparkLowLevel::ControlType::kVelocity, rev::spark::ClosedLoopSlot::kSlot0, ffVolts,
                                 rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
}

void ModuleIOSpark::SetTurnPosition(const frc::Rotation2d& rotation) {
    double setpoint = frc::InputModulus((rotation + zeroRotation_).Radians().value(), turnPIDMinInput, turnPIDMaxInput);
    turnController_.SetSetpoint(setpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
}
