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

#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/time.hpp>
#include <wpi/util/Alert.hpp>
#include <wpi/util/array.hpp>

#include "subsystems/vision/VisionIO.h"

class Vision : public wpi::cmd::SubsystemBase {
public:
    using VisionConsumer = std::function<void(const wpi::math::Pose2d& visionRobotPoseMeters, wpi::units::second_t timestampSeconds,
                                              const wpi::util::array<double, 3>& visionMeasurementStdDevs)>;

    template<std::derived_from<VisionIO>... IO>
    explicit Vision(VisionConsumer consumer, std::unique_ptr<IO>... io)
        : consumer_(std::move(consumer)) {
        (io_.push_back(std::move(io)), ...);

        // Initialize inputs
        inputs_.resize(io_.size());

        // Initialize disconnected alerts
        for (size_t i = 0; i < io_.size(); i++) {
            disconnectedAlerts_.push_back(std::make_unique<wpi::util::Alert>("Vision/Camera" + std::to_string(i) + "/Disconnected",
                                                                             "Vision camera " + std::to_string(i) + " is disconnected.",
                                                                             wpi::util::Alert::Level::MEDIUM));
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    wpi::math::Rotation2d GetTargetX(size_t cameraIndex) const;

    void Periodic() override;

private:
    VisionConsumer consumer_;
    std::vector<std::unique_ptr<VisionIO>> io_;
    std::vector<VisionIOInputs> inputs_;
    std::vector<std::unique_ptr<wpi::util::Alert>> disconnectedAlerts_;
};
