// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/ModuleIOTalonFXS.h"

#include <stdexcept>

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <units/frequency.h>

#include "subsystems/drive/Drive.h"
#include "subsystems/drive/PhoenixOdometryThread.h"
#include "util/PhoenixUtil.h"

using namespace PhoenixUtil;

ModuleIOTalonFXS::ModuleIOTalonFXS(const ModuleConstants& constants)
    : driveTalon_(constants.DriveMotorId, TunerConstants::kCANBus)
    , turnTalon_(constants.SteerMotorId, TunerConstants::kCANBus)
    , candi_(constants.EncoderId, TunerConstants::kCANBus) {
    // Configure drive motor
    auto driveConfig = constants.DriveMotorInitialConfigs;
    switch (constants.DriveMotorType) {
        case swerve::DriveMotorArrangement::TalonFXS_NEO_JST:
            driveConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::NEO_JST;
            break;
        case swerve::DriveMotorArrangement::TalonFXS_VORTEX_JST:
            driveConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::VORTEX_JST;
            break;
        default:
            driveConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::Disabled;
            break;
    }
    driveConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.ExternalFeedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;
    TryUntilOk(5, [&] { return driveTalon_.GetConfigurator().Apply(driveConfig, 0.25_s); });
    TryUntilOk(5, [&] { return driveTalon_.SetPosition(0_tr, 0.25_s); });

    // Configure turn motor
    configs::TalonFXSConfiguration turnConfig;
    switch (constants.SteerMotorType) {
        case swerve::SteerMotorArrangement::TalonFXS_Minion_JST:
            turnConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::Minion_JST;
            break;
        case swerve::SteerMotorArrangement::TalonFXS_NEO_JST:
            turnConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::NEO_JST;
            break;
        case swerve::SteerMotorArrangement::TalonFXS_VORTEX_JST:
            turnConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::VORTEX_JST;
            break;
        case swerve::SteerMotorArrangement::TalonFXS_NEO550_JST:
            turnConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::NEO550_JST;
            break;
        case swerve::SteerMotorArrangement::TalonFXS_Brushed_AB:
        case swerve::SteerMotorArrangement::TalonFXS_Brushed_AC:
        case swerve::SteerMotorArrangement::TalonFXS_Brushed_BC:
            turnConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::Brushed_DC;
            break;
        default:
            turnConfig.Commutation.MotorArrangement = signals::MotorArrangementValue::Disabled;
            break;
    }
    switch (constants.SteerMotorType) {
        case swerve::SteerMotorArrangement::TalonFXS_Brushed_AC:
            turnConfig.Commutation.BrushedMotorWiring = signals::BrushedMotorWiringValue::Leads_A_and_C;
            break;
        case swerve::SteerMotorArrangement::TalonFXS_Brushed_BC:
            turnConfig.Commutation.BrushedMotorWiring = signals::BrushedMotorWiringValue::Leads_B_and_C;
            break;
        default:
            turnConfig.Commutation.BrushedMotorWiring = signals::BrushedMotorWiringValue::Leads_A_and_B;
            break;
    }
    turnConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.ExternalFeedback.FeedbackRemoteSensorID = constants.EncoderId;
    switch (constants.FeedbackSource) {
        case swerve::SteerFeedbackType::RemoteCANdiPWM1:
            turnConfig.ExternalFeedback.ExternalFeedbackSensorSource = signals::ExternalFeedbackSensorSourceValue::RemoteCANdiPWM1;
            break;
        case swerve::SteerFeedbackType::FusedCANdiPWM1:
            turnConfig.ExternalFeedback.ExternalFeedbackSensorSource = signals::ExternalFeedbackSensorSourceValue::FusedCANdiPWM1;
            break;
        case swerve::SteerFeedbackType::SyncCANdiPWM1:
            turnConfig.ExternalFeedback.ExternalFeedbackSensorSource = signals::ExternalFeedbackSensorSourceValue::SyncCANdiPWM1;
            break;
        default:
            throw std::runtime_error(
                "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFXS (CANdi PWM 1). "
                "Please check the AdvantageKit documentation for more information on alternative configurations: "
                "https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
    }
    turnConfig.ExternalFeedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = units::turns_per_second_t{100.0 / constants.SteerMotorGearRatio};
    turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100_s;
    turnConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t{0.12 * constants.SteerMotorGearRatio};
    turnConfig.MotionMagic.MotionMagicExpo_kA = ctre::unit::volts_per_turn_per_second_squared_t{0.1};
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;
    TryUntilOk(5, [&] { return turnTalon_.GetConfigurator().Apply(turnConfig, 0.25_s); });

    // Configure CANdi
    configs::CANdiConfiguration candiConfig = constants.EncoderInitialConfigs;
    candiConfig.PWM1.AbsoluteSensorOffset = constants.EncoderOffset;
    candiConfig.PWM1.SensorDirection = constants.EncoderInverted;
    candi_.GetConfigurator().Apply(candiConfig);

    // Create timestamp queue
    timestampQueue_ = PhoenixOdometryThread::GetInstance().MakeTimestampQueue();

    // Create drive and turn odometry queues
    drivePositionQueue_ = PhoenixOdometryThread::GetInstance().RegisterSignal(drivePosition_);
    turnPositionQueue_ = PhoenixOdometryThread::GetInstance().RegisterSignal(turnPosition_);

    // Configure periodic frames
    BaseStatusSignal::SetUpdateFrequencyForAll(units::hertz_t{Drive::GetOdometryFrequency()}, drivePosition_, turnPosition_);
    BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, driveVelocity_, driveAppliedVolts_, driveCurrent_, turnAbsolutePosition_, turnVelocity_,
                                               turnAppliedVolts_, turnCurrent_);
    hardware::ParentDevice::OptimizeBusUtilizationForAll(driveTalon_, turnTalon_);
}

void ModuleIOTalonFXS::UpdateInputs(ModuleIOInputs& inputs) {
    // Refresh all signals
    auto driveStatus = BaseStatusSignal::RefreshAll(drivePosition_, driveVelocity_, driveAppliedVolts_, driveCurrent_);
    auto turnStatus = BaseStatusSignal::RefreshAll(turnPosition_, turnVelocity_, turnAppliedVolts_, turnCurrent_);
    auto turnEncoderStatus = BaseStatusSignal::RefreshAll(turnAbsolutePosition_);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce_.Calculate(driveStatus.IsOK());
    inputs.drivePositionRad = units::radian_t{drivePosition_.GetValue()}.value();
    inputs.driveVelocityRadPerSec = units::radians_per_second_t{driveVelocity_.GetValue()}.value();
    inputs.driveAppliedVolts = driveAppliedVolts_.GetValue().value();
    inputs.driveCurrentAmps = driveCurrent_.GetValue().value();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce_.Calculate(turnStatus.IsOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce_.Calculate(turnEncoderStatus.IsOK());
    inputs.turnAbsolutePosition = frc::Rotation2d{turnAbsolutePosition_.GetValue()};
    inputs.turnPosition = frc::Rotation2d{turnPosition_.GetValue()};
    inputs.turnVelocityRadPerSec = units::radians_per_second_t{turnVelocity_.GetValue()}.value();
    inputs.turnAppliedVolts = turnAppliedVolts_.GetValue().value();
    inputs.turnCurrentAmps = turnCurrent_.GetValue().value();

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue_->Drain();
    inputs.odometryDrivePositionsRad.clear();
    for (double value : drivePositionQueue_->Drain()) {
        inputs.odometryDrivePositionsRad.push_back(units::radian_t{units::turn_t{value}}.value());
    }
    inputs.odometryTurnPositions.clear();
    for (double value : turnPositionQueue_->Drain()) {
        inputs.odometryTurnPositions.push_back(frc::Rotation2d{units::turn_t{value}});
    }
}

void ModuleIOTalonFXS::SetDriveOpenLoop(double output) {
    driveTalon_.SetControl(voltageRequest_.WithOutput(units::volt_t{output}));
}

void ModuleIOTalonFXS::SetTurnOpenLoop(double output) {
    turnTalon_.SetControl(voltageRequest_.WithOutput(units::volt_t{output}));
}

void ModuleIOTalonFXS::SetDriveVelocity(double velocityRadPerSec) {
    units::turns_per_second_t velocityRotPerSec{units::radians_per_second_t{velocityRadPerSec}};
    driveTalon_.SetControl(velocityVoltageRequest_.WithVelocity(velocityRotPerSec));
}

void ModuleIOTalonFXS::SetTurnPosition(const frc::Rotation2d& rotation) {
    turnTalon_.SetControl(positionVoltageRequest_.WithPosition(rotation.Radians()));
}
