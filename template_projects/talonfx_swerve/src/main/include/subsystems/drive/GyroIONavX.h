// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <memory>

#include <studica/AHRS.h>

#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/OdometryQueue.h"

/** IO implementation for NavX. */
class GyroIONavX : public GyroIO {
public:
    GyroIONavX();

    void UpdateInputs(GyroIOInputs& inputs) override;

private:
    studica::AHRS navX_;
    std::shared_ptr<OdometryQueue> yawPositionQueue_;
    std::shared_ptr<OdometryQueue> yawTimestampQueue_;
};
