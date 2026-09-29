#include "pch.h"

#include "akit/mechanism/LoggedMechanismObject2d.h"

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

namespace akit::mechanism {
    using namespace units::literals;

    LoggedMechanismObject2d::LoggedMechanismObject2d(std::string_view name)
        : name_{name} {}

    const std::string& LoggedMechanismObject2d::GetName() const {
        return name_;
    }

    void LoggedMechanismObject2d::Update(std::shared_ptr<nt::NetworkTable> table) {
        std::scoped_lock lock(mutex_);
        table_ = table;
        UpdateEntries(table_);
        for (const auto& [name, object] : objects_) {
            object->Update(table_->GetSubTable(name));
        }
    }

    void LoggedMechanismObject2d::LogOutput(const LogTable& table) const {
        std::scoped_lock lock(mutex_);
        LogEntries(table);
        for (const auto& [name, object] : objects_) {
            object->LogOutput(table.GetSubtable(name));
        }
    }

    std::vector<frc::Pose3d> LoggedMechanismObject2d::Generate3dMechanism(const frc::Pose3d& seed) const {
        std::scoped_lock lock(mutex_);
        std::vector<frc::Pose3d> poses;
        for (const auto& [name, object] : objects_) {
            const frc::Rotation3d pitchFromAngle{0_rad, units::degree_t{-object->GetAngle()}, 0_rad};
            const frc::Pose3d pose{seed.Translation(), seed.Rotation().RotateBy(pitchFromAngle)};
            poses.push_back(pose);

            const frc::Pose3d childSeed =
                pose.TransformBy(frc::Transform3d{frc::Translation3d{units::meter_t{object->GetObject2dRange()}, 0_m, 0_m}, frc::Rotation3d{}});
            const auto childPoses = object->Generate3dMechanism(childSeed);
            poses.insert(poses.end(), childPoses.begin(), childPoses.end());
        }
        return poses;
    }
} // namespace akit::mechanism
