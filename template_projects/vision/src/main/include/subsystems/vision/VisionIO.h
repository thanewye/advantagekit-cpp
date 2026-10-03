// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <span>
#include <string_view>
#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/util/struct/Struct.hpp>

/** Represents the angle to a simple target, not used for pose estimation. */
struct TargetObservation {
    wpi::math::Rotation2d tx{};
    wpi::math::Rotation2d ty{};
};

enum class PoseObservationType { kMegatag1, kMegatag2, kPhotonVision };

/** Represents a robot pose sample used for pose estimation. */
struct PoseObservation {
    double timestamp = 0.0;
    wpi::math::Pose3d pose{};
    double ambiguity = 0.0;
    int tagCount = 0;
    double averageTagDistance = 0.0;
    PoseObservationType type = PoseObservationType::kMegatag1;
};

struct VisionIOInputs {
    bool connected = false;
    TargetObservation latestTargetObservation{};
    std::vector<PoseObservation> poseObservations{};
    std::vector<int> tagIds{};
};

class VisionIO {
public:
    virtual ~VisionIO() = default;

    virtual void UpdateInputs(VisionIOInputs& inputs) {}
};

template<> struct wpi::util::Struct<TargetObservation> {
    static constexpr std::string_view GetTypeName() { return "TargetObservation"; }
    static constexpr size_t GetSize() { return 2 * wpi::util::GetStructSize<wpi::math::Rotation2d>(); }
    static constexpr std::string_view GetSchema() { return "Rotation2d tx;Rotation2d ty;"; }

    static TargetObservation Unpack(std::span<const uint8_t> data) {
        constexpr size_t kTyOff = wpi::util::GetStructSize<wpi::math::Rotation2d>();
        return {wpi::util::UnpackStruct<wpi::math::Rotation2d, 0>(data), wpi::util::UnpackStruct<wpi::math::Rotation2d, kTyOff>(data)};
    }

    static void Pack(std::span<uint8_t> data, const TargetObservation& value) {
        constexpr size_t kTyOff = wpi::util::GetStructSize<wpi::math::Rotation2d>();
        wpi::util::PackStruct<0>(data, value.tx);
        wpi::util::PackStruct<kTyOff>(data, value.ty);
    }

    static void ForEachNested(std::invocable<std::string_view, std::string_view> auto fn) { wpi::util::ForEachStructSchema<wpi::math::Rotation2d>(fn); }
};

template<> struct wpi::util::Struct<PoseObservation> {
    static constexpr std::string_view GetTypeName() { return "PoseObservation"; }
    static constexpr size_t GetSize() { return 8 + wpi::util::GetStructSize<wpi::math::Pose3d>() + 8 + 4 + 8 + 4; }
    static constexpr std::string_view GetSchema() {
        return "double timestamp;Pose3d pose;double ambiguity;int32 tagCount;double averageTagDistance;enum {MEGATAG_1=0, MEGATAG_2=1, "
               "PHOTONVISION=2} int32 type;";
    }

    static PoseObservation Unpack(std::span<const uint8_t> data) {
        constexpr size_t kPoseOff = 8;
        constexpr size_t kAmbiguityOff = kPoseOff + wpi::util::GetStructSize<wpi::math::Pose3d>();
        constexpr size_t kTagCountOff = kAmbiguityOff + 8;
        constexpr size_t kAverageTagDistanceOff = kTagCountOff + 4;
        constexpr size_t kTypeOff = kAverageTagDistanceOff + 8;
        return {wpi::util::UnpackStruct<double, 0>(data),
                wpi::util::UnpackStruct<wpi::math::Pose3d, kPoseOff>(data),
                wpi::util::UnpackStruct<double, kAmbiguityOff>(data),
                wpi::util::UnpackStruct<int32_t, kTagCountOff>(data),
                wpi::util::UnpackStruct<double, kAverageTagDistanceOff>(data),
                static_cast<PoseObservationType>(wpi::util::UnpackStruct<int32_t, kTypeOff>(data))};
    }

    static void Pack(std::span<uint8_t> data, const PoseObservation& value) {
        constexpr size_t kPoseOff = 8;
        constexpr size_t kAmbiguityOff = kPoseOff + wpi::util::GetStructSize<wpi::math::Pose3d>();
        constexpr size_t kTagCountOff = kAmbiguityOff + 8;
        constexpr size_t kAverageTagDistanceOff = kTagCountOff + 4;
        constexpr size_t kTypeOff = kAverageTagDistanceOff + 8;
        wpi::util::PackStruct<0>(data, value.timestamp);
        wpi::util::PackStruct<kPoseOff>(data, value.pose);
        wpi::util::PackStruct<kAmbiguityOff>(data, value.ambiguity);
        wpi::util::PackStruct<kTagCountOff>(data, static_cast<int32_t>(value.tagCount));
        wpi::util::PackStruct<kAverageTagDistanceOff>(data, value.averageTagDistance);
        wpi::util::PackStruct<kTypeOff>(data, static_cast<int32_t>(value.type));
    }

    static void ForEachNested(std::invocable<std::string_view, std::string_view> auto fn) { wpi::util::ForEachStructSchema<wpi::math::Pose3d>(fn); }
};

static_assert(wpi::util::StructSerializable<TargetObservation>);
static_assert(wpi::util::StructSerializable<PoseObservation>);
