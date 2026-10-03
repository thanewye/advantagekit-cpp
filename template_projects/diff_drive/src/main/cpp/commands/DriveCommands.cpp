// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "commands/DriveCommands.h"

#include <cstdio>
#include <memory>
#include <vector>

#include <wpi/commands2/Commands.hpp>
#include <wpi/drive/DifferentialDrive.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/system/Timer.hpp>

namespace {
    constexpr double DEADBAND = 0.1;
    constexpr double FF_RAMP_RATE = 0.1; // Volts/Sec
} // namespace

wpi::cmd::CommandPtr DriveCommands::ArcadeDrive(Drive* drive, std::function<double()> xSupplier, std::function<double()> zSupplier) {
    return wpi::cmd::Run(
        [drive, xSupplier = std::move(xSupplier), zSupplier = std::move(zSupplier)] {
            // Apply deadband
            double x = wpi::math::ApplyDeadband(xSupplier(), DEADBAND);
            double z = wpi::math::ApplyDeadband(zSupplier(), DEADBAND);

            // Calculate speeds
            auto speeds = wpi::DifferentialDrive::ArcadeDriveIK(x, z, true);

            // Apply output
            drive->RunClosedLoop(speeds.left * DriveConstants::maxSpeedMetersPerSec, speeds.right * DriveConstants::maxSpeedMetersPerSec);
        },
        {drive});
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
            timer->Restart();
        }),

        // Accelerate and gather data
        wpi::cmd::Run(
            [=] {
                double voltage = timer->Get().value() * FF_RAMP_RATE;
                drive->RunOpenLoop(voltage, voltage);
                velocitySamples->push_back(drive->GetCharacterizationVelocity());
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
