#include "pch.h"

#include "akit/mechanism/LoggedMechanismObject2d.h"

#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/telemetry/TelemetryTable.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>

namespace akit::mechanism {
    using namespace wpi::units::literals;

    LoggedMechanismObject2d::LoggedMechanismObject2d(std::string_view name)
        : name_{name} {}

    const std::string& LoggedMechanismObject2d::GetName() const {
        return name_;
    }

    void LoggedMechanismObject2d::LogTo(wpi::telemetry::TelemetryTable& table) const {
        std::scoped_lock lock(mutex_);
        for (const auto& [name, object] : objects_) {
            table.Log(name, *object);
        }
    }

    void LoggedMechanismObject2d::LogOutput(const LogTable& table) const {
        std::scoped_lock lock(mutex_);
        LogEntries(table);
        for (const auto& [name, object] : objects_) {
            object->LogOutput(table.GetSubtable(name));
        }
    }

    std::vector<wpi::math::Pose3d> LoggedMechanismObject2d::Generate3dMechanism(const wpi::math::Pose3d& seed) const {
        std::scoped_lock lock(mutex_);
        std::vector<wpi::math::Pose3d> poses;
        for (const auto& [name, object] : objects_) {
            const wpi::math::Rotation3d pitchFromAngle{0_rad, wpi::units::degree_t{-object->GetAngle()}, 0_rad};
            const wpi::math::Pose3d pose{seed.Translation(), seed.Rotation().RotateBy(pitchFromAngle)};
            poses.push_back(pose);

            const wpi::math::Pose3d childSeed = pose.TransformBy(
                wpi::math::Transform3d{wpi::math::Translation3d{wpi::units::meter_t{object->GetObject2dRange()}, 0_m, 0_m}, wpi::math::Rotation3d{}});
            const auto childPoses = object->Generate3dMechanism(childSeed);
            poses.insert(poses.end(), childPoses.begin(), childPoses.end());
        }
        return poses;
    }
} // namespace akit::mechanism
