// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <string_view>

#include <photon/PhotonCamera.h>
#include <wpi/math/geometry/Transform3d.hpp>

#include "subsystems/vision/VisionIO.h"

/** IO implementation for real PhotonVision hardware. */
class VisionIOPhotonVision : public VisionIO {
public:
    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    VisionIOPhotonVision(std::string_view name, const wpi::math::Transform3d& robotToCamera);

    void UpdateInputs(VisionIOInputs& inputs) override;

protected:
    photon::PhotonCamera camera_;
    wpi::math::Transform3d robotToCamera_;
};
