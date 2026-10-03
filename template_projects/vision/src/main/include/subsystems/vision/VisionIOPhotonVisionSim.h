// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <string_view>

#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/VisionSystemSim.h>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>

#include "subsystems/vision/VisionIOPhotonVision.h"

/** IO implementation for physics sim using PhotonVision simulator. */
class VisionIOPhotonVisionSim : public VisionIOPhotonVision {
public:
    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    VisionIOPhotonVisionSim(std::string_view name, const wpi::math::Transform3d& robotToCamera, std::function<wpi::math::Pose2d()> poseSupplier);

    void UpdateInputs(VisionIOInputs& inputs) override;

private:
    inline static std::unique_ptr<photon::VisionSystemSim> visionSim;

    std::function<wpi::math::Pose2d()> poseSupplier_;
    std::unique_ptr<photon::PhotonCameraSim> cameraSim_;
};
