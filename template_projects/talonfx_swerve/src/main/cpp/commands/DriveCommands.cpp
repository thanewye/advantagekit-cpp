// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "commands/DriveCommands.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <vector>

#include <wpi/commands2/Commands.hpp>
#include <wpi/driverstation/MatchState.hpp>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/math/controller/ProfiledPIDController.hpp>
#include <wpi/math/filter/SlewRateLimiter.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Transform2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/trajectory/TrapezoidProfile.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/system/Timer.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>

namespace {
    constexpr double DEADBAND = 0.1;
    constexpr double ANGLE_KP = 5.0;
    constexpr double ANGLE_KD = 0.4;
    constexpr double ANGLE_MAX_VELOCITY = 8.0;
    constexpr double ANGLE_MAX_ACCELERATION = 20.0;
    constexpr double FF_START_DELAY = 2.0;             // Secs
    constexpr double FF_RAMP_RATE = 0.1;               // Volts/Sec
    constexpr double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    constexpr double WHEEL_RADIUS_RAMP_RATE = 0.05;    // Rad/Sec^2

    wpi::math::Translation2d GetLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = wpi::math::ApplyDeadband(std::hypot(x, y), DEADBAND);
        wpi::math::Rotation2d linearDirection{wpi::units::radian_t{std::atan2(y, x)}};

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return wpi::math::Pose2d{wpi::math::Translation2d{}, linearDirection}
            .TransformBy(wpi::math::Transform2d{wpi::units::meter_t{linearMagnitude}, 0_m, wpi::math::Rotation2d{}})
            .Translation();
    }

    struct WheelRadiusCharacterizationState {
        std::array<double, 4> positions{};
        wpi::math::Rotation2d lastAngle{};
        double gyroDelta = 0.0;
    };
} // namespace

wpi::cmd::CommandPtr DriveCommands::JoystickDrive(Drive* drive, std::function<double()> xSupplier, std::function<double()> ySupplier,
                                                  std::function<double()> omegaSupplier) {
    return wpi::cmd::Run(
        [drive, xSupplier = std::move(xSupplier), ySupplier = std::move(ySupplier), omegaSupplier = std::move(omegaSupplier)] {
            // Get linear velocity
            wpi::math::Translation2d linearVelocity = GetLinearVelocityFromJoysticks(xSupplier(), ySupplier());

            // Apply rotation deadband
            double omega = wpi::math::ApplyDeadband(omegaSupplier(), DEADBAND);

            // Square rotation value for more precise control
            omega = std::copysign(omega * omega, omega);

            // Convert to field relative speeds & send command
            wpi::math::ChassisVelocities speeds{wpi::units::meters_per_second_t{linearVelocity.X().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                                wpi::units::meters_per_second_t{linearVelocity.Y().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                                wpi::units::radians_per_second_t{omega * drive->GetMaxAngularSpeedRadPerSec()}};
            bool isFlipped = wpi::MatchState::GetAlliance().has_value() && wpi::MatchState::GetAlliance().value() == wpi::Alliance::RED;
            drive->RunVelocity(speeds.ToRobotRelative(isFlipped ? drive->GetRotation() + wpi::math::Rotation2d{wpi::units::radian_t{std::numbers::pi}}
                                                                : drive->GetRotation()));
        },
        {drive});
}

wpi::cmd::CommandPtr DriveCommands::JoystickDriveAtAngle(Drive* drive, std::function<double()> xSupplier, std::function<double()> ySupplier,
                                                         std::function<wpi::math::Rotation2d()> rotationSupplier) {
    // Create PID controller
    auto angleController = std::make_shared<wpi::math::ProfiledPIDController<wpi::units::radians>>(
        ANGLE_KP, 0.0, ANGLE_KD,
        wpi::math::TrapezoidProfile<wpi::units::radians>::Constraints{wpi::units::radians_per_second_t{ANGLE_MAX_VELOCITY},
                                                                      wpi::units::radians_per_second_squared_t{ANGLE_MAX_ACCELERATION}});
    angleController->EnableContinuousInput(wpi::units::radian_t{-std::numbers::pi}, wpi::units::radian_t{std::numbers::pi});

    // Construct command
    return wpi::cmd::Run(
               [drive, angleController, xSupplier = std::move(xSupplier), ySupplier = std::move(ySupplier), rotationSupplier = std::move(rotationSupplier)] {
                   // Get linear velocity
                   wpi::math::Translation2d linearVelocity = GetLinearVelocityFromJoysticks(xSupplier(), ySupplier());

                   // Calculate angular speed
                   double omega = angleController->Calculate(drive->GetRotation().Radians(), rotationSupplier().Radians());

                   // Convert to field relative speeds & send command
                   wpi::math::ChassisVelocities speeds{wpi::units::meters_per_second_t{linearVelocity.X().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                                       wpi::units::meters_per_second_t{linearVelocity.Y().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                                       wpi::units::radians_per_second_t{omega}};
                   bool isFlipped = wpi::MatchState::GetAlliance().has_value() && wpi::MatchState::GetAlliance().value() == wpi::Alliance::RED;
                   drive->RunVelocity(speeds.ToRobotRelative(isFlipped ? drive->GetRotation() + wpi::math::Rotation2d{wpi::units::radian_t{std::numbers::pi}}
                                                                       : drive->GetRotation()));
               },
               {drive})

        // Reset PID controller when command starts
        .BeforeStarting([drive, angleController] { angleController->Reset(drive->GetRotation().Radians()); });
}

wpi::cmd::CommandPtr DriveCommands::FeedforwardCharacterization(Drive* drive) {
    auto velocitySamples = std::make_shared<std::vector<double>>();
    auto voltageSamples = std::make_shared<std::vector<double>>();
    auto timer = std::make_shared<wpi::Timer>();

    return wpi::cmd::Sequence(
        // Reset data
        wpi::cmd::RunOnce([=] {
            velocitySamples->clear();
            voltageSamples->clear();
        }),

        // Allow modules to orient
        wpi::cmd::Run([=] { drive->RunCharacterization(0.0); }, {drive}).WithTimeout(wpi::units::second_t{FF_START_DELAY}),

        // Start timer
        wpi::cmd::RunOnce([=] { timer->Restart(); }),

        // Accelerate and gather data
        wpi::cmd::Run(
            [=] {
                double voltage = timer->Get().value() * FF_RAMP_RATE;
                drive->RunCharacterization(voltage);
                velocitySamples->push_back(drive->GetFFCharacterizationVelocity());
                voltageSamples->push_back(voltage);
            },
            {drive})

            // When cancelled, calculate and print results
            .FinallyDo([=] {
                size_t n = velocitySamples->size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (size_t i = 0; i < n; i++) {
                    sumX += (*velocitySamples)[i];
                    sumY += (*voltageSamples)[i];
                    sumXY += (*velocitySamples)[i] * (*voltageSamples)[i];
                    sumX2 += (*velocitySamples)[i] * (*velocitySamples)[i];
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                std::printf("********** Drive FF Characterization Results **********\n");
                std::printf("\tkS: %.5f\n", kS);
                std::printf("\tkV: %.5f\n", kV);
            }));
}

wpi::cmd::CommandPtr DriveCommands::WheelRadiusCharacterization(Drive* drive) {
    auto limiter =
        std::make_shared<wpi::math::SlewRateLimiter<wpi::units::radians_per_second>>(wpi::units::radians_per_second_squared_t{WHEEL_RADIUS_RAMP_RATE});
    auto state = std::make_shared<WheelRadiusCharacterizationState>();

    return wpi::cmd::Parallel(
        // Drive control sequence
        wpi::cmd::Sequence(
            // Reset acceleration limiter
            wpi::cmd::RunOnce([=] { limiter->Reset(0_rad_per_s); }),

            // Turn in place, accelerating up to full speed
            wpi::cmd::Run(
                [=] {
                    auto speed = limiter->Calculate(wpi::units::radians_per_second_t{WHEEL_RADIUS_MAX_VELOCITY});
                    drive->RunVelocity(wpi::math::ChassisVelocities{0_mps, 0_mps, speed});
                },
                {drive})),

        // Measurement sequence
        wpi::cmd::Sequence(
            // Wait for modules to fully orient before starting measurement
            wpi::cmd::Wait(1_s),

            // Record starting measurement
            wpi::cmd::RunOnce([=] {
                state->positions = drive->GetWheelRadiusCharacterizationPositions();
                state->lastAngle = drive->GetRotation();
                state->gyroDelta = 0.0;
            }),

            // Update gyro delta
            wpi::cmd::Run([=] {
                auto rotation = drive->GetRotation();
                state->gyroDelta += std::abs((rotation - state->lastAngle).Radians().value());
                state->lastAngle = rotation;
            })

                // When cancelled, calculate and print results
                .FinallyDo([=] {
                    auto positions = drive->GetWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (size_t i = 0; i < 4; i++) {
                        wheelDelta += std::abs(positions[i] - state->positions[i]) / 4.0;
                    }
                    double wheelRadius = (state->gyroDelta * Drive::DRIVE_BASE_RADIUS) / wheelDelta;

                    std::printf("********** Wheel Radius Characterization Results **********\n");
                    std::printf("\tWheel Delta: %.3f radians\n", wheelDelta);
                    std::printf("\tGyro Delta: %.3f radians\n", state->gyroDelta);
                    std::printf("\tWheel Radius: %.3f meters, %.3f inches\n", wheelRadius, wpi::units::inch_t{wpi::units::meter_t{wheelRadius}}.value());
                })));
}
