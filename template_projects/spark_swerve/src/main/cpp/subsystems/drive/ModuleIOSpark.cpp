// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/ModuleIOSpark.h"

#include <array>
#include <functional>

#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>
#include <wpi/math/util/MathUtil.hpp>

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/SparkOdometryThread.h"
#include "util/SparkUtil.h"

using namespace DriveConstants;
using namespace SparkUtil;

namespace {
    wpi::math::Rotation2d ZeroRotationForModule(int module) {
        switch (module) {
        case 0:
            return frontLeftZeroRotation;
        case 1:
            return frontRightZeroRotation;
        case 2:
            return backLeftZeroRotation;
        case 3:
            return backRightZeroRotation;
        default:
            return wpi::math::Rotation2d{};
        }
    }

    int DriveCanIdForModule(int module) {
        switch (module) {
        case 0:
            return frontLeftDriveCanId;
        case 1:
            return frontRightDriveCanId;
        case 2:
            return backLeftDriveCanId;
        case 3:
            return backRightDriveCanId;
        default:
            return 0;
        }
    }

    int TurnCanIdForModule(int module) {
        switch (module) {
        case 0:
            return frontLeftTurnCanId;
        case 1:
            return frontRightTurnCanId;
        case 2:
            return backLeftTurnCanId;
        case 3:
            return backRightTurnCanId;
        default:
            return 0;
        }
    }
} // namespace

ModuleIOSpark::ModuleIOSpark(int module)
    : zeroRotation_(ZeroRotationForModule(module))
    , driveSpark_(wpi::CANPort::CAN_S0, DriveCanIdForModule(module), rev::spark::SparkFlex::MotorType::kBrushless)
    , turnSpark_(wpi::CANPort::CAN_S0, TurnCanIdForModule(module), rev::spark::SparkMax::MotorType::kBrushless) {
    // Configure drive motor
    rev::spark::SparkFlexConfig driveConfig;
    driveConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake).SmartCurrentLimit(driveMotorCurrentLimit).VoltageCompensation(12.0);
    driveConfig.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
        .Pid(driveKp * driveEncoderVelocityFactor, 0.0, driveKd * driveEncoderVelocityFactor);
    driveConfig.signals.PrimaryEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(static_cast<int>(1000.0 / odometryFrequency))
        .PrimaryEncoderVelocityAlwaysOn(true)
        .PrimaryEncoderVelocityPeriodMs(20)
        .AppliedOutputPeriodMs(20)
        .BusVoltagePeriodMs(20)
        .OutputCurrentPeriodMs(20);
    TryUntilOk(driveSpark_, 5, [&] { return driveSpark_.Configure(driveConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });
    TryUntilOk(driveSpark_, 5, [&] { return driveEncoder_.SetPosition(0.0); });

    // Configure turn motor
    rev::spark::SparkMaxConfig turnConfig;
    turnConfig.Inverted(turnInverted)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(turnMotorCurrentLimit)
        .VoltageCompensation(12.0);
    turnConfig.absoluteEncoder.Inverted(turnEncoderInverted);
    turnConfig.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .PositionWrappingEnabled(true)
        .Pid(turnKp * turnEncoderPositionFactor, 0.0, turnKd * turnEncoderPositionFactor);
    turnConfig.signals.AbsoluteEncoderPositionAlwaysOn(true)
        .AbsoluteEncoderPositionPeriodMs(static_cast<int>(1000.0 / odometryFrequency))
        .AbsoluteEncoderVelocityAlwaysOn(true)
        .AbsoluteEncoderVelocityPeriodMs(20)
        .AppliedOutputPeriodMs(20)
        .BusVoltagePeriodMs(20)
        .OutputCurrentPeriodMs(20);
    TryUntilOk(turnSpark_, 5, [&] { return turnSpark_.Configure(turnConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters); });

    // Create odometry queues
    timestampQueue_ = SparkOdometryThread::GetInstance().MakeTimestampQueue();
    drivePositionQueue_ = SparkOdometryThread::GetInstance().RegisterSignal(
        [this] { return driveEncoder_.GetPosition().Map([](double value) { return value * driveEncoderPositionFactor; }); });
    turnPositionQueue_ = SparkOdometryThread::GetInstance().RegisterSignal(
        [this] { return turnEncoder_.GetPosition().Map([](double value) { return value * turnEncoderPositionFactor; }); });
}

void ModuleIOSpark::UpdateInputs(ModuleIOInputs& inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    IfOk([&] { return driveEncoder_.GetPosition(); }, [&](double value) { inputs.drivePositionRad = value * driveEncoderPositionFactor; });
    IfOk([&] { return driveEncoder_.GetVelocity(); }, [&](double value) { inputs.driveVelocityRadPerSec = value * driveEncoderVelocityFactor; });
    const std::array<std::function<rev::util::Signal<double>()>, 2> driveVoltageSuppliers{[&] { return driveSpark_.GetAppliedOutput(); },
                                                                                          [&] { return driveSpark_.GetBusVoltage(); }};
    IfOk(driveVoltageSuppliers, [&](const std::vector<double>& values) { inputs.driveAppliedVolts = values[0] * values[1]; });
    IfOk([&] { return driveSpark_.GetOutputCurrent(); }, [&](double value) { inputs.driveCurrentAmps = value; });
    inputs.driveConnected = driveConnectedDebounce_.Calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    IfOk([&] { return turnEncoder_.GetPosition(); },
         [&](double value) { inputs.turnPosition = wpi::math::Rotation2d{wpi::units::radian_t{value * turnEncoderPositionFactor}} - zeroRotation_; });
    IfOk([&] { return turnEncoder_.GetVelocity(); }, [&](double value) { inputs.turnVelocityRadPerSec = value * turnEncoderVelocityFactor; });
    const std::array<std::function<rev::util::Signal<double>()>, 2> turnVoltageSuppliers{[&] { return turnSpark_.GetAppliedOutput(); },
                                                                                         [&] { return turnSpark_.GetBusVoltage(); }};
    IfOk(turnVoltageSuppliers, [&](const std::vector<double>& values) { inputs.turnAppliedVolts = values[0] * values[1]; });
    IfOk([&] { return turnSpark_.GetOutputCurrent(); }, [&](double value) { inputs.turnCurrentAmps = value; });
    inputs.turnConnected = turnConnectedDebounce_.Calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue_->Drain();
    inputs.odometryDrivePositionsRad = drivePositionQueue_->Drain();
    inputs.odometryTurnPositions.clear();
    for (double value : turnPositionQueue_->Drain()) {
        inputs.odometryTurnPositions.push_back(wpi::math::Rotation2d{wpi::units::radian_t{value}} - zeroRotation_);
    }
}

void ModuleIOSpark::SetDriveOpenLoop(double output) {
    driveSpark_.SetVoltage(wpi::units::volt_t{output});
}

void ModuleIOSpark::SetTurnOpenLoop(double output) {
    turnSpark_.SetVoltage(wpi::units::volt_t{output});
}

void ModuleIOSpark::SetDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * ((velocityRadPerSec > 0) - (velocityRadPerSec < 0)) + driveKv * velocityRadPerSec;
    driveController_.SetSetpoint(velocityRadPerSec / driveEncoderVelocityFactor, rev::spark::SparkLowLevel::ControlType::kVelocity,
                                 rev::spark::ClosedLoopSlot::kSlot0, ffVolts, rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
}

void ModuleIOSpark::SetTurnPosition(const wpi::math::Rotation2d& rotation) {
    double setpoint = wpi::math::InputModulus((rotation + zeroRotation_).Radians().value(), turnPIDMinInput, turnPIDMaxInput);
    turnController_.SetSetpoint(setpoint / turnEncoderPositionFactor, rev::spark::SparkLowLevel::ControlType::kPosition);
}
