// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>
#include <string_view>
#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/nt/DoubleArrayTopic.hpp>
#include <wpi/nt/DoubleTopic.hpp>

#include "subsystems/vision/VisionIO.h"

/** IO implementation for real Limelight hardware. */
class VisionIOLimelight : public VisionIO {
public:
    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
     */
    VisionIOLimelight(std::string_view name, std::function<wpi::math::Rotation2d()> rotationSupplier);

    void UpdateInputs(VisionIOInputs& inputs) override;

private:
    /** Parses the 3D pose from a Limelight botpose array. */
    static wpi::math::Pose3d ParsePose(const std::vector<double>& rawLLArray);

    std::function<wpi::math::Rotation2d()> rotationSupplier_;
    wpi::nt::DoubleArrayPublisher orientationPublisher_;

    wpi::nt::DoubleSubscriber latencySubscriber_;
    wpi::nt::DoubleSubscriber txSubscriber_;
    wpi::nt::DoubleSubscriber tySubscriber_;
    wpi::nt::DoubleArraySubscriber megatag1Subscriber_;
    wpi::nt::DoubleArraySubscriber megatag2Subscriber_;
};
