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

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/Timer.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "subsystems/drive/DriveConstants.h"

namespace {
    constexpr double DEADBAND = 0.1;
    constexpr double ANGLE_KP = 5.0;
    constexpr double ANGLE_KD = 0.4;
    constexpr double ANGLE_MAX_VELOCITY = 8.0;
    constexpr double ANGLE_MAX_ACCELERATION = 20.0;
    constexpr double FF_START_DELAY = 2.0;            // Secs
    constexpr double FF_RAMP_RATE = 0.1;              // Volts/Sec
    constexpr double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    constexpr double WHEEL_RADIUS_RAMP_RATE = 0.05;    // Rad/Sec^2

    frc::Translation2d GetLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = frc::ApplyDeadband(std::hypot(x, y), DEADBAND);
        frc::Rotation2d linearDirection{units::radian_t{std::atan2(y, x)}};

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return frc::Pose2d{frc::Translation2d{}, linearDirection}
            .TransformBy(frc::Transform2d{units::meter_t{linearMagnitude}, 0_m, frc::Rotation2d{}})
            .Translation();
    }

    struct WheelRadiusCharacterizationState {
        std::array<double, 4> positions{};
        frc::Rotation2d lastAngle{};
        double gyroDelta = 0.0;
    };
} // namespace

frc2::CommandPtr DriveCommands::JoystickDrive(Drive* drive, std::function<double()> xSupplier, std::function<double()> ySupplier,
                                              std::function<double()> omegaSupplier) {
    return frc2::cmd::Run(
        [drive, xSupplier = std::move(xSupplier), ySupplier = std::move(ySupplier), omegaSupplier = std::move(omegaSupplier)] {
            // Get linear velocity
            frc::Translation2d linearVelocity = GetLinearVelocityFromJoysticks(xSupplier(), ySupplier());

            // Apply rotation deadband
            double omega = frc::ApplyDeadband(omegaSupplier(), DEADBAND);

            // Square rotation value for more precise control
            omega = std::copysign(omega * omega, omega);

            // Convert to field relative speeds & send command
            frc::ChassisSpeeds speeds{units::meters_per_second_t{linearVelocity.X().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                      units::meters_per_second_t{linearVelocity.Y().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                      units::radians_per_second_t{omega * drive->GetMaxAngularSpeedRadPerSec()}};
            bool isFlipped = frc::DriverStation::GetAlliance().has_value() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed;
            drive->RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                speeds, isFlipped ? drive->GetRotation() + frc::Rotation2d{units::radian_t{std::numbers::pi}} : drive->GetRotation()));
        },
        {drive});
}

frc2::CommandPtr DriveCommands::JoystickDriveAtAngle(Drive* drive, std::function<double()> xSupplier, std::function<double()> ySupplier,
                                                     std::function<frc::Rotation2d()> rotationSupplier) {
    // Create PID controller
    auto angleController = std::make_shared<frc::ProfiledPIDController<units::radians>>(
        ANGLE_KP, 0.0, ANGLE_KD,
        frc::TrapezoidProfile<units::radians>::Constraints{units::radians_per_second_t{ANGLE_MAX_VELOCITY},
                                                           units::radians_per_second_squared_t{ANGLE_MAX_ACCELERATION}});
    angleController->EnableContinuousInput(units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});

    // Construct command
    return frc2::cmd::Run(
               [drive, angleController, xSupplier = std::move(xSupplier), ySupplier = std::move(ySupplier),
                rotationSupplier = std::move(rotationSupplier)] {
                   // Get linear velocity
                   frc::Translation2d linearVelocity = GetLinearVelocityFromJoysticks(xSupplier(), ySupplier());

                   // Calculate angular speed
                   double omega = angleController->Calculate(drive->GetRotation().Radians(), rotationSupplier().Radians());

                   // Convert to field relative speeds & send command
                   frc::ChassisSpeeds speeds{units::meters_per_second_t{linearVelocity.X().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                             units::meters_per_second_t{linearVelocity.Y().value() * drive->GetMaxLinearSpeedMetersPerSec()},
                                             units::radians_per_second_t{omega}};
                   bool isFlipped =
                       frc::DriverStation::GetAlliance().has_value() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed;
                   drive->RunVelocity(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                       speeds, isFlipped ? drive->GetRotation() + frc::Rotation2d{units::radian_t{std::numbers::pi}} : drive->GetRotation()));
               },
               {drive})

        // Reset PID controller when command starts
        .BeforeStarting([drive, angleController] { angleController->Reset(drive->GetRotation().Radians()); });
}

frc2::CommandPtr DriveCommands::FeedforwardCharacterization(Drive* drive) {
    auto velocitySamples = std::make_shared<std::vector<double>>();
    auto voltageSamples = std::make_shared<std::vector<double>>();
    auto timer = std::make_shared<frc::Timer>();

    return frc2::cmd::Sequence(
        // Reset data
        frc2::cmd::RunOnce([=] {
            velocitySamples->clear();
            voltageSamples->clear();
        }),

        // Allow modules to orient
        frc2::cmd::Run([=] { drive->RunCharacterization(0.0); }, {drive}).WithTimeout(units::second_t{FF_START_DELAY}),

        // Start timer
        frc2::cmd::RunOnce([=] { timer->Restart(); }),

        // Accelerate and gather data
        frc2::cmd::Run(
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

frc2::CommandPtr DriveCommands::WheelRadiusCharacterization(Drive* drive) {
    auto limiter = std::make_shared<frc::SlewRateLimiter<units::radians_per_second>>(units::radians_per_second_squared_t{WHEEL_RADIUS_RAMP_RATE});
    auto state = std::make_shared<WheelRadiusCharacterizationState>();

    return frc2::cmd::Parallel(
        // Drive control sequence
        frc2::cmd::Sequence(
            // Reset acceleration limiter
            frc2::cmd::RunOnce([=] { limiter->Reset(0_rad_per_s); }),

            // Turn in place, accelerating up to full speed
            frc2::cmd::Run(
                [=] {
                    auto speed = limiter->Calculate(units::radians_per_second_t{WHEEL_RADIUS_MAX_VELOCITY});
                    drive->RunVelocity(frc::ChassisSpeeds{0_mps, 0_mps, speed});
                },
                {drive})),

        // Measurement sequence
        frc2::cmd::Sequence(
            // Wait for modules to fully orient before starting measurement
            frc2::cmd::Wait(1_s),

            // Record starting measurement
            frc2::cmd::RunOnce([=] {
                state->positions = drive->GetWheelRadiusCharacterizationPositions();
                state->lastAngle = drive->GetRotation();
                state->gyroDelta = 0.0;
            }),

            // Update gyro delta
            frc2::cmd::Run([=] {
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
                    double wheelRadius = (state->gyroDelta * DriveConstants::driveBaseRadius) / wheelDelta;

                    std::printf("********** Wheel Radius Characterization Results **********\n");
                    std::printf("\tWheel Delta: %.3f radians\n", wheelDelta);
                    std::printf("\tGyro Delta: %.3f radians\n", state->gyroDelta);
                    std::printf("\tWheel Radius: %.3f meters, %.3f inches\n", wheelRadius, units::inch_t{units::meter_t{wheelRadius}}.value());
                })));
}
