// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <vector>

#include <frc/Alert.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/time.h>
#include <wpi/array.h>

#include "subsystems/vision/VisionIO.h"

class Vision : public frc2::SubsystemBase {
public:
    using VisionConsumer =
        std::function<void(const frc::Pose2d& visionRobotPoseMeters, units::second_t timestampSeconds, const wpi::array<double, 3>& visionMeasurementStdDevs)>;

    template<std::derived_from<VisionIO>... IO>
    explicit Vision(VisionConsumer consumer, std::unique_ptr<IO>... io)
        : consumer_(std::move(consumer)) {
        (io_.push_back(std::move(io)), ...);

        // Initialize inputs
        inputs_.resize(io_.size());

        // Initialize disconnected alerts
        for (size_t i = 0; i < io_.size(); i++) {
            disconnectedAlerts_.push_back(
                std::make_unique<frc::Alert>("Vision camera " + std::to_string(i) + " is disconnected.", frc::Alert::AlertType::kWarning));
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    frc::Rotation2d GetTargetX(size_t cameraIndex) const;

    void Periodic() override;

private:
    VisionConsumer consumer_;
    std::vector<std::unique_ptr<VisionIO>> io_;
    std::vector<VisionIOInputs> inputs_;
    std::vector<std::unique_ptr<frc::Alert>> disconnectedAlerts_;
};
