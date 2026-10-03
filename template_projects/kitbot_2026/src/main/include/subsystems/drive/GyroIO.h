// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <wpi/math/geometry/Rotation2d.hpp>

struct GyroIOInputs {
    bool connected = false;
    wpi::math::Rotation2d yawPosition{};
    double yawVelocityRadPerSec = 0.0;
};

class GyroIO {
public:
    virtual ~GyroIO() = default;

    virtual void UpdateInputs(GyroIOInputs& inputs) {}
};
