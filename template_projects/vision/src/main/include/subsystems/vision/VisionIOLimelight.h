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

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>

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
    VisionIOLimelight(std::string_view name, std::function<frc::Rotation2d()> rotationSupplier);

    void UpdateInputs(VisionIOInputs& inputs) override;

private:
    /** Parses the 3D pose from a Limelight botpose array. */
    static frc::Pose3d ParsePose(const std::vector<double>& rawLLArray);

    std::function<frc::Rotation2d()> rotationSupplier_;
    nt::DoubleArrayPublisher orientationPublisher_;

    nt::DoubleSubscriber latencySubscriber_;
    nt::DoubleSubscriber txSubscriber_;
    nt::DoubleSubscriber tySubscriber_;
    nt::DoubleArraySubscriber megatag1Subscriber_;
    nt::DoubleArraySubscriber megatag2Subscriber_;
};
