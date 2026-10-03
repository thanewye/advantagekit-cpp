// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/vision/VisionIOPhotonVisionSim.h"

#include <photon/simulation/SimCameraProperties.h>

#include "subsystems/vision/VisionConstants.h"

using namespace VisionConstants;

VisionIOPhotonVisionSim::VisionIOPhotonVisionSim(std::string_view name, const wpi::math::Transform3d& robotToCamera,
                                                 std::function<wpi::math::Pose2d()> poseSupplier)
    : VisionIOPhotonVision(name, robotToCamera)
    , poseSupplier_(std::move(poseSupplier)) {
    // Initialize vision sim
    if (!visionSim) {
        visionSim = std::make_unique<photon::VisionSystemSim>("main");
        visionSim->AddAprilTags(aprilTagLayout);
    }

    // Add sim camera
    photon::SimCameraProperties cameraProperties;
    cameraSim_ = std::make_unique<photon::PhotonCameraSim>(&camera_, cameraProperties, aprilTagLayout);
    visionSim->AddCamera(cameraSim_.get(), robotToCamera);
}

void VisionIOPhotonVisionSim::UpdateInputs(VisionIOInputs& inputs) {
    visionSim->Update(poseSupplier_());
    VisionIOPhotonVision::UpdateInputs(inputs);
}
