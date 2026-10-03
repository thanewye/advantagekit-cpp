// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/ModuleIOTalonFX.h"

#include <stdexcept>

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <wpi/units/frequency.hpp>

#include "subsystems/drive/Drive.h"
#include "subsystems/drive/PhoenixOdometryThread.h"
#include "util/PhoenixUtil.h"

using namespace PhoenixUtil;

ModuleIOTalonFX::ModuleIOTalonFX(const ModuleConstants& constants)
    : constants_(constants)
    , driveTalon_(constants.DriveMotorId, TunerConstants::kCANBus)
    , turnTalon_(constants.SteerMotorId, TunerConstants::kCANBus)
    , cancoder_(constants.EncoderId, TunerConstants::kCANBus) {
    // Configure drive motor
    auto driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;
    TryUntilOk(5, [&] { return driveTalon_.GetConfigurator().Apply(driveConfig, 0.25_s); });
    TryUntilOk(5, [&] { return driveTalon_.SetPosition(0_tr, 0.25_s); });

    // Configure turn motor
    configs::TalonFXConfiguration turnConfig;
    turnConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    switch (constants.FeedbackSource) {
    case swerve::SteerFeedbackType::RemoteCANcoder:
        turnConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
        break;
    case swerve::SteerFeedbackType::FusedCANcoder:
        turnConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
        break;
    case swerve::SteerFeedbackType::SyncCANcoder:
        turnConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::SyncCANcoder;
        break;
    default:
        throw std::runtime_error(
            "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the "
            "AdvantageKit documentation for more information on alternative configurations: "
            "https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
    }
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = wpi::units::turns_per_second_t{100.0 / constants.SteerMotorGearRatio};
    turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100_s;
    turnConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t{0.12 * constants.SteerMotorGearRatio};
    turnConfig.MotionMagic.MotionMagicExpo_kA = ctre::unit::volts_per_turn_per_second_squared_t{0.1};
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;
    TryUntilOk(5, [&] { return turnTalon_.GetConfigurator().Apply(turnConfig, 0.25_s); });

    // Configure CANCoder
    configs::CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted ? signals::SensorDirectionValue::Clockwise_Positive : signals::SensorDirectionValue::CounterClockwise_Positive;
    cancoder_.GetConfigurator().Apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue_ = PhoenixOdometryThread::GetInstance().MakeTimestampQueue();

    // Create drive and turn odometry queues
    drivePositionQueue_ = PhoenixOdometryThread::GetInstance().RegisterSignal(drivePosition_);
    turnPositionQueue_ = PhoenixOdometryThread::GetInstance().RegisterSignal(turnPosition_);

    // Configure periodic frames
    BaseStatusSignal::SetUpdateFrequencyForAll(wpi::units::hertz_t{Drive::GetOdometryFrequency()}, drivePosition_, turnPosition_);
    BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, driveVelocity_, driveAppliedVolts_, driveCurrent_, turnAbsolutePosition_, turnVelocity_,
                                               turnAppliedVolts_, turnCurrent_);
    hardware::ParentDevice::OptimizeBusUtilizationForAll(driveTalon_, turnTalon_);
}

void ModuleIOTalonFX::UpdateInputs(ModuleIOInputs& inputs) {
    // Refresh all signals
    auto driveStatus = BaseStatusSignal::RefreshAll(drivePosition_, driveVelocity_, driveAppliedVolts_, driveCurrent_);
    auto turnStatus = BaseStatusSignal::RefreshAll(turnPosition_, turnVelocity_, turnAppliedVolts_, turnCurrent_);
    auto turnEncoderStatus = BaseStatusSignal::RefreshAll(turnAbsolutePosition_);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce_.Calculate(driveStatus.IsOK());
    inputs.drivePositionRad = wpi::units::radian_t{drivePosition_.GetValue()}.value();
    inputs.driveVelocityRadPerSec = wpi::units::radians_per_second_t{driveVelocity_.GetValue()}.value();
    inputs.driveAppliedVolts = driveAppliedVolts_.GetValue().value();
    inputs.driveCurrentAmps = driveCurrent_.GetValue().value();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce_.Calculate(turnStatus.IsOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce_.Calculate(turnEncoderStatus.IsOK());
    inputs.turnAbsolutePosition = wpi::math::Rotation2d{turnAbsolutePosition_.GetValue()};
    inputs.turnPosition = wpi::math::Rotation2d{turnPosition_.GetValue()};
    inputs.turnVelocityRadPerSec = wpi::units::radians_per_second_t{turnVelocity_.GetValue()}.value();
    inputs.turnAppliedVolts = turnAppliedVolts_.GetValue().value();
    inputs.turnCurrentAmps = turnCurrent_.GetValue().value();

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue_->Drain();
    inputs.odometryDrivePositionsRad.clear();
    for (double value : drivePositionQueue_->Drain()) {
        inputs.odometryDrivePositionsRad.push_back(wpi::units::radian_t{wpi::units::turn_t{value}}.value());
    }
    inputs.odometryTurnPositions.clear();
    for (double value : turnPositionQueue_->Drain()) {
        inputs.odometryTurnPositions.push_back(wpi::math::Rotation2d{wpi::units::turn_t{value}});
    }
}

void ModuleIOTalonFX::SetDriveOpenLoop(double output) {
    switch (constants_.DriveMotorClosedLoopOutput) {
    case swerve::ClosedLoopOutputType::Voltage:
        driveTalon_.SetControl(voltageRequest_.WithOutput(wpi::units::volt_t{output}));
        break;
    case swerve::ClosedLoopOutputType::TorqueCurrentFOC:
        driveTalon_.SetControl(torqueCurrentRequest_.WithOutput(wpi::units::ampere_t{output}));
        break;
    }
}

void ModuleIOTalonFX::SetTurnOpenLoop(double output) {
    switch (constants_.SteerMotorClosedLoopOutput) {
    case swerve::ClosedLoopOutputType::Voltage:
        turnTalon_.SetControl(voltageRequest_.WithOutput(wpi::units::volt_t{output}));
        break;
    case swerve::ClosedLoopOutputType::TorqueCurrentFOC:
        turnTalon_.SetControl(torqueCurrentRequest_.WithOutput(wpi::units::ampere_t{output}));
        break;
    }
}

void ModuleIOTalonFX::SetDriveVelocity(double velocityRadPerSec) {
    wpi::units::turns_per_second_t velocityRotPerSec{wpi::units::radians_per_second_t{velocityRadPerSec}};
    switch (constants_.DriveMotorClosedLoopOutput) {
    case swerve::ClosedLoopOutputType::Voltage:
        driveTalon_.SetControl(velocityVoltageRequest_.WithVelocity(velocityRotPerSec));
        break;
    case swerve::ClosedLoopOutputType::TorqueCurrentFOC:
        driveTalon_.SetControl(velocityTorqueCurrentRequest_.WithVelocity(velocityRotPerSec));
        break;
    }
}

void ModuleIOTalonFX::SetTurnPosition(const wpi::math::Rotation2d& rotation) {
    switch (constants_.SteerMotorClosedLoopOutput) {
    case swerve::ClosedLoopOutputType::Voltage:
        turnTalon_.SetControl(positionVoltageRequest_.WithPosition(rotation.Radians()));
        break;
    case swerve::ClosedLoopOutputType::TorqueCurrentFOC:
        turnTalon_.SetControl(positionTorqueCurrentRequest_.WithPosition(rotation.Radians()));
        break;
    }
}
