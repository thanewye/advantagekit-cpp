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

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/struct/Struct.h>

/** Represents the angle to a simple target, not used for pose estimation. */
struct TargetObservation {
    frc::Rotation2d tx{};
    frc::Rotation2d ty{};
};

enum class PoseObservationType { kMegatag1, kMegatag2, kPhotonVision };

/** Represents a robot pose sample used for pose estimation. */
struct PoseObservation {
    double timestamp = 0.0;
    frc::Pose3d pose{};
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

template<> struct wpi::Struct<TargetObservation> {
    static constexpr std::string_view GetTypeName() { return "TargetObservation"; }
    static constexpr size_t GetSize() { return 2 * wpi::GetStructSize<frc::Rotation2d>(); }
    static constexpr std::string_view GetSchema() { return "Rotation2d tx;Rotation2d ty;"; }

    static TargetObservation Unpack(std::span<const uint8_t> data) {
        constexpr size_t kTyOff = wpi::GetStructSize<frc::Rotation2d>();
        return {wpi::UnpackStruct<frc::Rotation2d, 0>(data), wpi::UnpackStruct<frc::Rotation2d, kTyOff>(data)};
    }

    static void Pack(std::span<uint8_t> data, const TargetObservation& value) {
        constexpr size_t kTyOff = wpi::GetStructSize<frc::Rotation2d>();
        wpi::PackStruct<0>(data, value.tx);
        wpi::PackStruct<kTyOff>(data, value.ty);
    }

    static void ForEachNested(std::invocable<std::string_view, std::string_view> auto fn) { wpi::ForEachStructSchema<frc::Rotation2d>(fn); }
};

template<> struct wpi::Struct<PoseObservation> {
    static constexpr std::string_view GetTypeName() { return "PoseObservation"; }
    static constexpr size_t GetSize() { return 8 + wpi::GetStructSize<frc::Pose3d>() + 8 + 4 + 8 + 4; }
    static constexpr std::string_view GetSchema() {
        return "double timestamp;Pose3d pose;double ambiguity;int32 tagCount;double averageTagDistance;enum {MEGATAG_1=0, MEGATAG_2=1, "
               "PHOTONVISION=2} int32 type;";
    }

    static PoseObservation Unpack(std::span<const uint8_t> data) {
        constexpr size_t kPoseOff = 8;
        constexpr size_t kAmbiguityOff = kPoseOff + wpi::GetStructSize<frc::Pose3d>();
        constexpr size_t kTagCountOff = kAmbiguityOff + 8;
        constexpr size_t kAverageTagDistanceOff = kTagCountOff + 4;
        constexpr size_t kTypeOff = kAverageTagDistanceOff + 8;
        return {wpi::UnpackStruct<double, 0>(data),
                wpi::UnpackStruct<frc::Pose3d, kPoseOff>(data),
                wpi::UnpackStruct<double, kAmbiguityOff>(data),
                wpi::UnpackStruct<int32_t, kTagCountOff>(data),
                wpi::UnpackStruct<double, kAverageTagDistanceOff>(data),
                static_cast<PoseObservationType>(wpi::UnpackStruct<int32_t, kTypeOff>(data))};
    }

    static void Pack(std::span<uint8_t> data, const PoseObservation& value) {
        constexpr size_t kPoseOff = 8;
        constexpr size_t kAmbiguityOff = kPoseOff + wpi::GetStructSize<frc::Pose3d>();
        constexpr size_t kTagCountOff = kAmbiguityOff + 8;
        constexpr size_t kAverageTagDistanceOff = kTagCountOff + 4;
        constexpr size_t kTypeOff = kAverageTagDistanceOff + 8;
        wpi::PackStruct<0>(data, value.timestamp);
        wpi::PackStruct<kPoseOff>(data, value.pose);
        wpi::PackStruct<kAmbiguityOff>(data, value.ambiguity);
        wpi::PackStruct<kTagCountOff>(data, static_cast<int32_t>(value.tagCount));
        wpi::PackStruct<kAverageTagDistanceOff>(data, value.averageTagDistance);
        wpi::PackStruct<kTypeOff>(data, static_cast<int32_t>(value.type));
    }

    static void ForEachNested(std::invocable<std::string_view, std::string_view> auto fn) { wpi::ForEachStructSchema<frc::Pose3d>(fn); }
};

static_assert(wpi::StructSerializable<TargetObservation>);
static_assert(wpi::StructSerializable<PoseObservation>);
